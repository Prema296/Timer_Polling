In which created two functions such as 
Timer_set_cnt( uint32_t delay) : which is of void type to set the timer for the specified time. 
Timer_Frequency_Cofig (uint32_t timer) : which is of void type to configure the timer frequency. 
Then checking for the "TIM_FLAG_UPDATE " flag to get the timer elapsed time. 
Green LED will be turning off once this flag is set.
