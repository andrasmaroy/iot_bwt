#ifdef _ULPCC_ // Do not add anything above this def
/* This is a C version of the assembly example shipped in the ESP IDF at $IDF_PATH/examples/system/ulp */
#include <ulp_c.h>

unsigned next_edge, debounce_counter, debounce_max_count, edge_count, io_number;

void entry()
{ 
    unsigned rtc;
    if (io_number >= 16)
        /* read_io_high */
        rtc = READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 16, 2) >> (io_number - 16);
    else
        rtc = READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16) >> io_number;

    /* read_done */
    rtc &= 1;
    if (rtc != next_edge)
        debounce_counter = debounce_max_count;
    else
    {
        /* changed */
        if (debounce_counter)
            debounce_counter--;
        else
        {
            /* edge_detected */
            debounce_counter = debounce_max_count;
            next_edge = !next_edge;
            ++edge_count;
        }
    }
}
#endif // do not add anything after here
