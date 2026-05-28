#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <math.h>

/* =========================================================
 * FMAC registers (STM32H5)
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

/* FMAC CR bits */
#define FMAC_CR_START    (1U << 1)
#define FMAC_CR_RESET    (1U << 16)
#define FMAC_CR_WIEN     (1U << 8)   /* Write interrupt enable */
#define FMAC_CR_RIEN     (1U << 9)   /* Read interrupt enable */
#define FMAC_CR_OVFLIEN  (1U << 10)  /* Overflow interrupt enable */

/* FMAC SR bits */
#define FMAC_SR_YEMPTY   (1U << 3)
#define FMAC_SR_X1FULL   (1U << 0)

/* RCC */
#define RCC_AHB1ENR        (*(volatile uint32_t *)(0x40020C00UL + 0x48))
#define RCC_AHB1ENR_FMACEN (1U << 16)

/* FMAC IRQ番号 (STM32H5) */
#define FMAC_IRQ 115

/* FIR settings */
#define FIR_TAPS     64
#define FIR_BUF_SIZE 64

/* Stack sizes */
#define STACK_SIZE_FMAC 2048
#define STACK_SIZE_TASK 1024

/* Task priorities */
#define PRIO_FMAC  2
#define PRIO_TASKS 5

/* =========================================================
 * Globals
 * ========================================================= */
static struct k_sem fmac_done;

/* Task execution counters */
static volatile uint32_t cnt_a, cnt_b, cnt_c, cnt_d, cnt_e;

/* LED */
static const struct gpio_dt_spec led0 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

/* Stacks */
K_THREAD_STACK_DEFINE(fmac_stack, STACK_SIZE_FMAC);
K_THREAD_STACK_DEFINE(task_a_stack, STACK_SIZE_TASK);
K_THREAD_STACK_DEFINE(task_b_stack, STACK_SIZE_TASK);
K_THREAD_STACK_DEFINE(task_c_stack, STACK_SIZE_TASK);
K_THREAD_STACK_DEFINE(task_d_stack, STACK_SIZE_TASK);
K_THREAD_STACK_DEFINE(task_e_stack, STACK_SIZE_TASK);

static struct k_thread fmac_thread;
static struct k_thread task_a_thread, task_b_thread, task_c_thread,
                       task_d_thread, task_e_thread;

/* =========================================================
 * FMAC ISR
 * ========================================================= */
static void fmac_isr(void *arg)
{
    ARG_UNUSED(arg);
    /* 割り込みを無効化してセマフォをgive */
    FMAC_CR &= ~(FMAC_CR_RIEN);
    k_sem_give(&fmac_done);
}

/* =========================================================
 * FMAC init
 * ========================================================= */
static void fmac_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_FMACEN;

    FMAC_CR = FMAC_CR_RESET;
    FMAC_CR = 0;

    FMAC_X1BUFCFG = ((FIR_BUF_SIZE & 0xFF) << 8) | 0;
    FMAC_X2BUFCFG = ((FIR_TAPS     & 0xFF) << 8) | (FIR_BUF_SIZE & 0xFF);
    FMAC_YBUFCFG  = ((FIR_BUF_SIZE & 0xFF) << 8)
                  | ((FIR_BUF_SIZE + FIR_TAPS) & 0xFF);

    /* Load coefficients (LOAD_X2 = 0x02) */
    FMAC_PARAM = (0x02U << 24) | ((FIR_TAPS & 0xFF) << 8)
               | (FIR_BUF_SIZE & 0xFF);
    FMAC_CR = FMAC_CR_START;

    for (int i = 0; i < FIR_TAPS; i++) {
        FMAC_WDATA = (uint32_t)(int16_t)(32767 / FIR_TAPS);
    }
    while (FMAC_CR & FMAC_CR_START) {}
    FMAC_CR = 0;

    /* IRQ登録 */
    IRQ_CONNECT(FMAC_IRQ, 0, fmac_isr, NULL, 0);
    irq_enable(FMAC_IRQ);
}

/* =========================================================
 * FMAC task
 * ========================================================= */
static void task_fmac(void *a, void *b, void *c)
{
    fmac_init();
    printk("[FMAC] init done\n");

    uint32_t iter = 0;

    while (1) {
        /* スナップショット（演算前） */
        uint32_t a0 = cnt_a, b0 = cnt_b, c0 = cnt_c,
                 d0 = cnt_d, e0 = cnt_e;

        /* FIR start (CONVO_FIR = 0x08) */
        FMAC_PARAM = (0x08U << 24) | ((FIR_BUF_SIZE & 0xFF) << 8)
                   | (FIR_TAPS & 0xFF);
        FMAC_CR = FMAC_CR_START;

        /* Write input data */
        for (int i = 0; i < FIR_BUF_SIZE; i++) {
            while (FMAC_SR & FMAC_SR_X1FULL) {}
            int16_t s = (int16_t)(sinf(
                2.0f * 3.14159f * i / FIR_BUF_SIZE) * 32767.0f);
            FMAC_WDATA = (uint32_t)(uint16_t)s;
        }

        /* Enable read interrupt → ISRでsem give */
        FMAC_CR |= FMAC_CR_RIEN;

        /* Block until FMAC done ← ここでA~Eが動く */
        k_sem_take(&fmac_done, K_FOREVER);

        /* Read output */
        int16_t out0 = (int16_t)(FMAC_RDATA & 0xFFFF);
        for (int i = 1; i < FIR_BUF_SIZE; i++) {
            (void)FMAC_RDATA;
        }

        /* Log task execution counts during FMAC */
        iter++;
        printk("[FMAC] iter=%u out=%d "
               "A=%u B=%u C=%u D=%u E=%u\n",
               iter, out0,
               cnt_a - a0, cnt_b - b0, cnt_c - c0,
               cnt_d - d0, cnt_e - e0);

        gpio_pin_toggle_dt(&led0);
    }
}

/* =========================================================
 * CPU tasks A~E (same priority, different workloads)
 * ========================================================= */

/* Task A: very light */
static void task_a(void *a, void *b, void *c)
{
    while (1) {
        cnt_a++;
        k_yield();
    }
}

/* Task B: light arithmetic */
static void task_b(void *a, void *b, void *c)
{
    volatile uint32_t x = 1;
    while (1) {
        for (int i = 0; i < 100; i++) { x = x * 3 + 1; }
        cnt_b++;
        k_yield();
    }
}

/* Task C: medium - matrix 8x8 */
static void task_c(void *a, void *b, void *c)
{
    static volatile float mat[8][8];
    while (1) {
        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 8; j++)
                mat[i][j] = (float)(i * j + 1);
        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 8; j++)
                for (int k = 0; k < 8; k++)
                    mat[i][j] += mat[i][k] * mat[k][j];
        cnt_c++;
        k_yield();
    }
}

/* Task D: CRC32 */
static void task_d(void *a, void *b, void *c)
{
    static const uint8_t buf[256] = {0};
    while (1) {
        uint32_t crc = 0xFFFFFFFF;
        for (int i = 0; i < 256; i++) {
            crc ^= buf[i];
            for (int j = 0; j < 8; j++) {
                crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
            }
        }
        cnt_d++;
        k_yield();
    }
}

/* Task E: heavy - prime search */
static void task_e(void *a, void *b, void *c)
{
    while (1) {
        volatile uint32_t n = 9999991;
        bool prime = true;
        for (uint32_t i = 2; i * i <= n; i++) {
            if (n % i == 0) { prime = false; break; }
        }
        (void)prime;
        cnt_e++;
        k_yield();
    }
}

/* =========================================================
 * Main
 * ========================================================= */
int main(void)
{
    k_msleep(3000);
    printk("=== FMAC OFFLOAD DEMO ===\n");
    printk("Tasks A-E: same priority=%d\n", PRIO_TASKS);
    printk("FMAC task: priority=%d\n", PRIO_FMAC);

    k_sem_init(&fmac_done, 0, 1);

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);

    k_thread_create(&fmac_thread, fmac_stack,
                    K_THREAD_STACK_SIZEOF(fmac_stack),
                    task_fmac, NULL, NULL, NULL,
                    PRIO_FMAC, 0, K_NO_WAIT);
    k_thread_create(&task_a_thread, task_a_stack,
                    K_THREAD_STACK_SIZEOF(task_a_stack),
                    task_a, NULL, NULL, NULL,
                    PRIO_TASKS, 0, K_NO_WAIT);
    k_thread_create(&task_b_thread, task_b_stack,
                    K_THREAD_STACK_SIZEOF(task_b_stack),
                    task_b, NULL, NULL, NULL,
                    PRIO_TASKS, 0, K_NO_WAIT);
    k_thread_create(&task_c_thread, task_c_stack,
                    K_THREAD_STACK_SIZEOF(task_c_stack),
                    task_c, NULL, NULL, NULL,
                    PRIO_TASKS, 0, K_NO_WAIT);
    k_thread_create(&task_d_thread, task_d_stack,
                    K_THREAD_STACK_SIZEOF(task_d_stack),
                    task_d, NULL, NULL, NULL,
                    PRIO_TASKS, 0, K_NO_WAIT);
    k_thread_create(&task_e_thread, task_e_stack,
                    K_THREAD_STACK_SIZEOF(task_e_stack),
                    task_e, NULL, NULL, NULL,
                    PRIO_TASKS, 0, K_NO_WAIT);

    return 0;
}