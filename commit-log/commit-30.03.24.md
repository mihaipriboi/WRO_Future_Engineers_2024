# Commit - 30/03/2024

## Added servo functions and refactored the code

I added the control functions for the servo (MG90S) and calibrated its pulses. Also, now the execute function can get 2 types of strings: "{speed}" and "s{angle}", where the first one is a command which sets the motor speed (-100->100) and the second one sets the servo motor angle (-1->1).