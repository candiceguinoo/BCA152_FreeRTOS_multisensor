#ifndef DISPLAY_H
#define DISPLAY_H

// DisplayTask: sole owner of the SSD1306 OLED.
// It initializes the I2C bus and the panel itself,
// then draws the selected page.

void displayTask(void *parameter);

#endif // DISPLAY_H
