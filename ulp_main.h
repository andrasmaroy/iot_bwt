/*
    Put your ULP globals here you want visibility
    for your sketch. Add "ulp_" to the beginning
    of the variable name and must be size 'uint32_t'
*/
#include "Arduino.h"
// points to the entry function in counter.c.
extern uint32_t ulp_entry;
// pointer to counter in counter.c
extern uint32_t ulp_next_edge, ulp_debounce_counter, ulp_debounce_max_count, ulp_edge_count, ulp_io_number;
