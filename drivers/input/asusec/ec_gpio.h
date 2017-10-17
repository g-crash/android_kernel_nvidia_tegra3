#include <mach/irqs.h>

#define TEGRA_NR_GPIOS		INT_GPIO_NR

#if 0
static inline int gpio_to_irq(unsigned int gpio)
{
	/* SOC gpio */
	if (gpio < TEGRA_NR_GPIOS)
		return INT_GPIO_BASE + gpio;

    return -EINVAL;
}
//#define gpio_to_irq __gpio_to_irq
#endif

static inline int irq_to_gpio(unsigned int irq)
{
	/* SOC gpio */
	if ((irq >= INT_GPIO_BASE) && (irq < INT_GPIO_BASE + INT_GPIO_NR))
		return irq - INT_GPIO_BASE;

	/* we don't supply reverse mappings for non-SOC gpios */
	return -EINVAL;
}

