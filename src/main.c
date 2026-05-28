#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

/* =========================================================
 * FMACレジスタ定義 (STM32H5)
 * ========================================================= */
#define FMAC_BASE      0x40021400UL
#define FMAC_X1BUFCFG (*(volatile uint32_t *)(FMAC_BASE + 0x00))
#define FMAC_X2BUFCFG (*(volatile uint32_t *)(FMAC_BASE + 0x04))
#define FMAC_YBUFCFG  (*(volatile uint32_t *)(FMAC_BASE + 0x08))
#define FMAC_PARAM    (*(volatile uint32_t *)(FMAC_BASE + 0x0C))
#define FMAC_CR       (*(volatile uint32_t *)(FMAC_BASE + 0x10))
#define FMAC_SR       (*(volatile uint32_t *)(FMAC_BASE + 0x14))
#define FMAC_WDATA    (*(volatile uint32_t *)(FMAC_BASE + 0x18))
#define FMAC_RDATA    (*(volatile uint32_t *)(FMAC_BASE + 0x1C))

/* RCC: FMACクロック有効化 */
#define RCC_AHB1ENR       (*(volatile uint32_t *)(0x40020C00UL + 0x48))
#define RCC_AHB1ENR_FMACEN (1U << 16)

/* FIR設定 */
#define FIR_TAPS     16
#define FIR_BUF_SIZE 16

/* スタック */
#define STACK_SIZE 2048

/* LED */
static const struct gpio_dt_spec led0 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

K_THREAD_STACK_DEFINE(fmac_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(led_stack,  STACK_SIZE);

static struct k_thread fmac_thread, led_thread;

/* LEDタスクの実行カウンタ */
static volatile uint32_t led_count = 0;

/* ---- FMAC初期化 ---- */
static void fmac_init(void)
{
    /* クロック有効化 */
    RCC_AHB1ENR |= RCC_AHB1ENR_FMACEN;

    /* リセット */
    FMAC_CR = (1U << 16);
    FMAC_CR = 0;

    /* X1バッファ: base=0, size=FIR_BUF_SIZE */
    FMAC_X1BUFCFG = ((FIR_BUF_SIZE & 0xFF) << 8) | 0;

    /* X2バッファ（係数）: base=FIR_BUF_SIZE, size=FIR_TAPS */
    FMAC_X2BUFCFG = ((FIR_TAPS & 0xFF) << 8) | (FIR_BUF_SIZE & 0xFF);

    /* Yバッファ: base=FIR_BUF_SIZE+FIR_TAPS, size=FIR_BUF_SIZE */
    FMAC_YBUFCFG = ((FIR_BUF_SIZE & 0xFF) << 8)
                 | ((FIR_BUF_SIZE + FIR_TAPS) & 0xFF);

    /* 係数ロード（LOAD_X2 = 0x02） */
    FMAC_PARAM = (0x02U << 24) | ((FIR_TAPS & 0xFF) << 8)
               | (FIR_BUF_SIZE & 0xFF);
    FMAC_CR = (1U << 1); /* START */

    /* 均一係数（ローパス）をロード */
    for (int i = 0; i < FIR_TAPS; i++) {
        FMAC_WDATA = (uint32_t)(int16_t)(32767 / FIR_TAPS);
    }

    /* ロード完了待ち */
    while (FMAC_CR & (1U << 1)) {}
    FMAC_CR = 0;
}

/* ---- FMACタスク ---- */
static void task_fmac(void *a, void *b, void *c)
{
    fmac_init();
    printk("[FMAC] 初期化完了\n");

    uint32_t count = 0;

    while (1) {
        /* FIRフィルタ開始（CONVO_FIR = 0x08） */
        FMAC_PARAM = (0x08U << 24) | ((FIR_BUF_SIZE & 0xFF) << 8)
                   | (FIR_TAPS & 0xFF);
        FMAC_CR = (1U << 1); /* START */

        /* 入力データ書き込み（サイン波 Q1.15） */
        for (int i = 0; i < FIR_BUF_SIZE; i++) {
            while (FMAC_SR & (1U << 0)) { k_yield(); } /* XFULL待ち */
            int16_t s = (int16_t)(sinf(
                2.0f * 3.14159f * i / FIR_BUF_SIZE) * 32767.0f);
            FMAC_WDATA = (uint32_t)(uint16_t)s;
        }

        /* 出力読み出し */
        int16_t out0 = 0;
        for (int i = 0; i < FIR_BUF_SIZE; i++) {
            while (FMAC_SR & (1U << 3)) { k_yield(); } /* YEMPTY待ち */
            uint32_t r = FMAC_RDATA;
            if (i == 0) out0 = (int16_t)(r & 0xFFFF);
        }

        count++;
        printk("[FMAC] 演算%u回完了 out[0]=%d LED実行回数=%u\n",
               count, out0, led_count);

        k_msleep(200);
    }
}

/* ---- LEDタスク ---- */
static void task_led(void *a, void *b, void *c)
{
    while (1) {
        gpio_pin_toggle_dt(&led0);
        gpio_pin_toggle_dt(&led1);
        led_count++;
        printk("[LED] toggle count=%u\n", led_count);
        k_msleep(100);
    }
}

/* ---- メイン ---- */
int main(void)
{
    k_msleep(10000);
    printk("=== FMAC START ===\n");

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);

    k_thread_create(&fmac_thread, fmac_stack,
                    K_THREAD_STACK_SIZEOF(fmac_stack),
                    task_fmac, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
    k_thread_create(&led_thread, led_stack,
                    K_THREAD_STACK_SIZEOF(led_stack),
                    task_led,  NULL, NULL, NULL, 6, 0, K_NO_WAIT);

    return 0;
}
