#include "driver_st7789_rp2350.h"

#include "driver_st7789_interface_rp2350.h"

static st7789_handle_t gs_handle;

static uint8_t a_st7789_rp2350_fail(const char *message)
{
    st7789_interface_debug_print("st7789: %s failed.\n", message);
    if (gs_handle.inited != 0U)
    {
        (void)st7789_deinit(&gs_handle);
    }

    return 1;
}

uint8_t st7789_rp2350_init(void)
{
    uint8_t access;

    if (gs_handle.inited != 0U)
    {
        return 1;
    }

    DRIVER_ST7789_LINK_INIT(&gs_handle, st7789_handle_t);
    DRIVER_ST7789_LINK_SPI_INIT(&gs_handle, st7789_interface_spi_init);
    DRIVER_ST7789_LINK_SPI_DEINIT(&gs_handle, st7789_interface_spi_deinit);
    DRIVER_ST7789_LINK_SPI_WRITE_COMMAND(&gs_handle, st7789_interface_spi_write_cmd);
    DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_INIT(&gs_handle, st7789_interface_cmd_data_gpio_init);
    DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_DEINIT(&gs_handle, st7789_interface_cmd_data_gpio_deinit);
    DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_WRITE(&gs_handle, st7789_interface_cmd_data_gpio_write);
    DRIVER_ST7789_LINK_RESET_GPIO_INIT(&gs_handle, st7789_interface_reset_gpio_init);
    DRIVER_ST7789_LINK_RESET_GPIO_DEINIT(&gs_handle, st7789_interface_reset_gpio_deinit);
    DRIVER_ST7789_LINK_RESET_GPIO_WRITE(&gs_handle, st7789_interface_reset_gpio_write);
    DRIVER_ST7789_LINK_DELAY_MS(&gs_handle, st7789_interface_delay_ms);
    DRIVER_ST7789_LINK_DEBUG_PRINT(&gs_handle, st7789_interface_debug_print);

    if (st7789_init(&gs_handle) != 0U)
    {
        return a_st7789_rp2350_fail("init");
    }
    if (st7789_set_column(&gs_handle, 240U) != 0U)
    {
        return a_st7789_rp2350_fail("set column");
    }
    if (st7789_set_row(&gs_handle, 320U) != 0U)
    {
        return a_st7789_rp2350_fail("set row");
    }
    if (st7789_sleep_out(&gs_handle) != 0U)
    {
        return a_st7789_rp2350_fail("sleep out");
    }
    if (st7789_idle_mode_off(&gs_handle) != 0U)
    {
        return a_st7789_rp2350_fail("idle mode off");
    }
    if (st7789_normal_display_mode_on(&gs_handle) != 0U)
    {
        return a_st7789_rp2350_fail("normal display mode on");
    }
    if (st7789_display_inversion_off(&gs_handle) != 0U)
    {
        return a_st7789_rp2350_fail("display inversion off");
    }

    access = ST7789_ORDER_PAGE_TOP_TO_BOTTOM |
             ST7789_ORDER_COLUMN_LEFT_TO_RIGHT |
             ST7789_ORDER_PAGE_COLUMN_NORMAL |
             ST7789_ORDER_LINE_TOP_TO_BOTTOM |
             ST7789_ORDER_COLOR_RGB |
             ST7789_ORDER_REFRESH_LEFT_TO_RIGHT;
    if (st7789_set_memory_data_access_control(&gs_handle, access) != 0U)
    {
        return a_st7789_rp2350_fail("set memory access control");
    }
    if (st7789_set_interface_pixel_format(&gs_handle,
                                           ST7789_RGB_INTERFACE_COLOR_FORMAT_65K,
                                           ST7789_CONTROL_INTERFACE_COLOR_FORMAT_16_BIT) != 0U)
    {
        return a_st7789_rp2350_fail("set pixel format");
    }
    if (st7789_clear(&gs_handle) != 0U)
    {
        return a_st7789_rp2350_fail("clear");
    }
    if (st7789_display_on(&gs_handle) != 0U)
    {
        return a_st7789_rp2350_fail("display on");
    }

    return 0;
}

uint8_t st7789_rp2350_deinit(void)
{
    if (gs_handle.inited == 0U)
    {
        return 0;
    }

    return st7789_deinit(&gs_handle);
}

st7789_handle_t *st7789_rp2350_get_handle(void)
{
    return &gs_handle;
}
