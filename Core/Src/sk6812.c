#include "sk6812.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;

// Helper to check non-blocking delays
uint8_t check_elapsed_time(uint32_t *last_tick, uint32_t interval) {
    if (HAL_GetTick() - *last_tick >= interval) {
        *last_tick = HAL_GetTick();
        return 1;
    }
    return 0;
}

void LEDS_OFF(PixelRGB_t *colors, uint8_t num_of_leds) {
    for (uint8_t i = 0; i < num_of_leds; i++) {
        colors[i].color.r = 0;
        colors[i].color.g = 0;
        colors[i].color.b = 0;
    }
}

void RGB_COLORS(PixelRGB_t *colors, uint8_t num_of_leds, uint8_t red_hue, uint8_t blue_hue, uint8_t green_hue) {
    for (uint8_t i = 0; i < num_of_leds; i++) {
        colors[i].color.r = red_hue;
        colors[i].color.g = green_hue;
        colors[i].color.b = blue_hue;
    }
}

void SEND_DATA(PixelRGB_t *colors, uint32_t *dma_buffer, uint8_t num_of_leds, uint32_t high_time, uint32_t low_time) {
    uint32_t *buffer_pointer = dma_buffer;
    for (uint8_t i = 0; i < num_of_leds; i++) {
        for (int8_t j = 23; j >= 0; j--) {
            *buffer_pointer++ = (colors[i].data >> j) & 0x01 ? high_time : low_time;
        }
    }
    *buffer_pointer = 0; // Reset timing
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, dma_buffer, (num_of_leds * 24) + 1);
}

void TheaterChaseRGB(PixelRGB_t *pixels, uint32_t *dmaBuffer, uint8_t num_leds,
                     uint8_t red, uint8_t green, uint8_t blue,
                     uint8_t spacing, uint8_t iterations,
                     uint8_t delay_ms, uint32_t high_time, uint32_t low_time) {
    static uint32_t last_tick = 0;
    static uint8_t iter = 0, step = 0;

    if (check_elapsed_time(&last_tick, delay_ms)) {
        // Clear all LEDs
        for (uint8_t i = 0; i < num_leds; i++) {
            pixels[i].data = 0;
        }

        // Set LEDs in spacing intervals with RGB values
        for (uint8_t i = step; i < num_leds; i += spacing) {
            pixels[i].color.r = red;
            pixels[i].color.g = green;
            pixels[i].color.b = blue;
        }

        // Send updated data to LEDs
        SEND_DATA(pixels, dmaBuffer, num_leds, high_time, low_time);

        step = (step + 1) % spacing;
        if (step == 0) iter++;
        if (iter >= iterations) iter = 0;  // Reset after completion
    }
}

void CometTail(PixelRGB_t *pixels, uint32_t *dmaBuffer, uint8_t num_leds,
               uint32_t color, uint8_t tail_length, uint8_t delay_ms,
               uint32_t high_time, uint32_t low_time) {
    static uint32_t last_tick = 0;
    static uint8_t head = 0;

    if (check_elapsed_time(&last_tick, delay_ms)) {
        for (uint8_t i = 0; i < num_leds; i++) {
            int16_t distance = head - i;
            if (distance >= 0 && distance < tail_length) {
                uint8_t brightness = 255 - (distance * (255 / tail_length));
                pixels[i].color.r = ((color >> 16) & 0xFF) * brightness / 255;
                pixels[i].color.g = ((color >> 8) & 0xFF) * brightness / 255;
                pixels[i].color.b = (color & 0xFF) * brightness / 255;
            } else {
                pixels[i].data = 0;
            }
        }
        SEND_DATA(pixels, dmaBuffer, num_leds, high_time, low_time);
        head = (head + 1) % (num_leds + tail_length);
    }
}

void Twinkle(PixelRGB_t *pixels, uint32_t *dmaBuffer, uint8_t num_leds,
             uint32_t twinkle_color, uint8_t sparkles, uint8_t delay_ms,
             uint32_t high_time, uint32_t low_time) {
    static uint32_t last_tick = 0;

    if (check_elapsed_time(&last_tick, delay_ms)) {
        for (uint8_t i = 0; i < num_leds; i++) {
            pixels[i].color.r = 50;
            pixels[i].color.g = 50;
            pixels[i].color.b = 50;
        }
        uint8_t index = rand() % num_leds;
        pixels[index].data = twinkle_color;
        SEND_DATA(pixels, dmaBuffer, num_leds, high_time, low_time);
    }
}

uint32_t hsl_to_rgb(uint8_t h, uint8_t s, uint8_t l)
{
    uint8_t r, g, b;

    float hf = h / 255.0f; // Convert hue to a float in range 0.0 to 1.0
    float sf = s / 255.0f; // Saturation as a float
    float lf = l / 255.0f; // Lightness as a float

    float c = (1.0f - fabsf(2.0f * lf - 1.0f)) * sf; // Chroma
    float x = c * (1.0f - fabsf(fmodf(hf * 6.0f, 2.0f) - 1.0f));
    float m = lf - (c / 2.0f);

    float rf = 0, gf = 0, bf = 0;

    if (hf < 1.0f / 6.0f) {
        rf = c; gf = x; bf = 0;
    } else if (hf < 2.0f / 6.0f) {
        rf = x; gf = c; bf = 0;
    } else if (hf < 3.0f / 6.0f) {
        rf = 0; gf = c; bf = x;
    } else if (hf < 4.0f / 6.0f) {
        rf = 0; gf = x; bf = c;
    } else if (hf < 5.0f / 6.0f) {
        rf = x; gf = 0; bf = c;
    } else {
        rf = c; gf = 0; bf = x;
    }

    r = (uint8_t)((rf + m) * 255.0f);
    g = (uint8_t)((gf + m) * 255.0f);
    b = (uint8_t)((bf + m) * 255.0f);

    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; // Pack into 24-bit color
}


void RainbowFade(PixelRGB_t *pixels, uint32_t *dmaBuffer, uint8_t num_leds,
                 uint8_t delay_ms, uint32_t high_time, uint32_t low_time) {
    static uint32_t last_tick = 0;
    static uint8_t hue = 0;

    if (check_elapsed_time(&last_tick, delay_ms)) {
        for (uint8_t i = 0; i < num_leds; i++) {
            uint32_t color = hsl_to_rgb((hue + (i * 5)) % 255, 255, 128);
            pixels[i].color.r = (color >> 16) & 0xFF;
            pixels[i].color.g = (color >> 8) & 0xFF;
            pixels[i].color.b = color & 0xFF;
        }
        SEND_DATA(pixels, dmaBuffer, num_leds, high_time, low_time);
        hue++;
    }
}

void ColorWipe(PixelRGB_t *pixels, uint32_t *dmaBuffer, uint8_t num_leds,
               uint8_t delay_ms, uint32_t high_time, uint32_t low_time, uint16_t hold_time_ms) {
    static uint32_t last_tick = 0;
    static uint8_t index = 0;
    static uint8_t current_color_index = 0;
    static uint8_t hold_phase = 0; // Flag to indicate holding after a full wipe

    // Define the rainbow colors in an array (adjusted for b-r-g order)
    uint8_t rainbow_colors[][3] = {
        {0, 200, 0},   // Red
        {0, 200, 127}, // Orange
        {0, 200, 200}, // Yellow
        {0, 0, 200},   // Green
        {200, 0, 0},   // Blue
        {130, 0, 75},  // Indigo
        {200, 0, 148}  // Violet
    };
    const uint8_t num_colors = sizeof(rainbow_colors) / sizeof(rainbow_colors[0]);

    if (hold_phase) {
        // Hold phase: Keep the LEDs on the current color
        if (check_elapsed_time(&last_tick, hold_time_ms)) {
            hold_phase = 0; // End hold phase, start wiping next color
            current_color_index = (current_color_index + 1) % num_colors;
            index = 0; // Reset index for the next wipe
        }
        return; // Do nothing during hold
    }

    if (check_elapsed_time(&last_tick, delay_ms)) {
        if (index < num_leds) {
            // Set the current LED to the current color
            pixels[index].color.b = rainbow_colors[current_color_index][0];
            pixels[index].color.r = rainbow_colors[current_color_index][1];
            pixels[index].color.g = rainbow_colors[current_color_index][2];
            SEND_DATA(pixels, dmaBuffer, num_leds, high_time, low_time);
            index++;
        } else {
            // Transition to hold phase
            hold_phase = 1;
        }
    }
}

void WaveEffect(PixelRGB_t *pixels, uint32_t *dmaBuffer, uint8_t num_leds,
                uint32_t color, uint8_t wave_length, uint8_t delay_ms,
                uint32_t high_time, uint32_t low_time)
{
    static uint8_t offset = 0; // Wave offset for motion
    static uint32_t last_tick = 0;

    if (check_elapsed_time(&last_tick, delay_ms)) {
        for (uint8_t i = 0; i < num_leds; i++) {
            uint8_t brightness = 128 + 127 * sinf((2 * 3.14159f * (i + offset)) / wave_length);

            pixels[i].color.r = ((color >> 16) & 0xFF) * brightness / 255;
            pixels[i].color.g = ((color >> 8) & 0xFF) * brightness / 255;
            pixels[i].color.b = (color & 0xFF) * brightness / 255;
        }

        offset = (offset + 1) % wave_length; // Move the wave
        SEND_DATA(pixels, dmaBuffer, num_leds, high_time, low_time);
    }
}

void DodgersBlueWhiteWave(PixelRGB_t *pixels, uint32_t *dmaBuffer, uint8_t num_leds,
                          uint8_t wave_speed, uint32_t high_time, uint32_t low_time)
{
    static uint8_t wave_offset = 0; // Tracks the position of the wave
    static uint32_t last_tick = 0;  // Non-blocking timing for wave updates

    // Define Dodgers Blue color (RGB = 0, 0, 255)
    const uint8_t blue_r = 0;
    const uint8_t blue_g = 0;
    const uint8_t blue_b = 200;

    // Define White color (RGB = 255, 255, 255)
    const uint8_t white_r = 200;
    const uint8_t white_g = 200;
    const uint8_t white_b = 200;

    if (check_elapsed_time(&last_tick, wave_speed)) {
        for (uint8_t i = 0; i < num_leds; i++) {
            // Calculate wave position for alternating effect
            uint8_t position = (i + wave_offset) % num_leds;

            // Alternate blue and white based on position
            if (position % 2 == 0) {
                // Blue
                pixels[i].color.r = blue_r;
                pixels[i].color.g = blue_g;
                pixels[i].color.b = blue_b;
            } else {
                // White
                pixels[i].color.r = white_r;
                pixels[i].color.g = white_g;
                pixels[i].color.b = white_b;
            }
        }

        // Move the wave forward
        wave_offset = (wave_offset + 1) % num_leds;

        // Send the updated LED data
        SEND_DATA(pixels, dmaBuffer, num_leds, high_time, low_time);
    }
}




