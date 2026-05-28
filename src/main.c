#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

#define STACK_SIZE 1024

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

K_THREAD_STACK_DEFINE(task0_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(task1_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(task2_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(task3_stack, STACK_SIZE);

static struct k_thread task0_thread, task1_thread,
                       task2_thread, task3_thread;

/* タスク0: LED0を500msで点滅 */
static void task0(void *a, void *b, void *c)
{
    while (1) {
        gpio_pin_toggle_dt(&led0);
        printk("[Task0] LED0 toggle\n");
        k_msleep(500);
    }
}

/* タスク1: LED1を300msで点滅 */
static void task1(void *a, void *b, void *c)
{
    while (1) {
        gpio_pin_toggle_dt(&led1);
        printk("[Task1] LED1 toggle\n");
        k_msleep(300);
    }
}

/* タスク2: カウンタ演算 */
static void task2(void *a, void *b, void *c)
{
    uint32_t count = 0;
    while (1) {
        count++;
        if (count % 1000 == 0) {
            printk("[Task2] count=%u\n", count);
        }
        k_yield();
    }
}

/* タスク3: ログ出力 */
static void task3(void *a, void *b, void *c)
{
    uint32_t tick = 0;
    while (1) {
        printk("[Task3] uptime=%lldms tick=%u\n",
               k_uptime_get(), tick++);
        k_msleep(1000);
    }
}

int main(void)
{
    printk("=== start ===\n");

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);

    k_thread_create(&task0_thread, task0_stack,
                    K_THREAD_STACK_SIZEOF(task0_stack),
                    task0, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
    k_thread_create(&task1_thread, task1_stack,
                    K_THREAD_STACK_SIZEOF(task1_stack),
                    task1, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
    k_thread_create(&task2_thread, task2_stack,
                    K_THREAD_STACK_SIZEOF(task2_stack),
                    task2, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
    k_thread_create(&task3_thread, task3_stack,
                    K_THREAD_STACK_SIZEOF(task3_stack),
                    task3, NULL, NULL, NULL, 5, 0, K_NO_WAIT);

    return 0;
}
