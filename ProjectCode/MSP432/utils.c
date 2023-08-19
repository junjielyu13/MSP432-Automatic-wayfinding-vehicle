#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

bool time_out(int time){
    return (current_time>=time);
}