// LANC timing constants at 9600 baud (in microseconds)
const uint32_t BIT_TIME = 104;
const uint32_t HALF_BIT_TIME = 52;
const uint32_t SYNC_GAP_MIN = 5000; // 5ms sync preamble threshold

volatile enum { SEARCHING_SYNC, WAITING_START, PROCESSING_BYTE } lanc_state = SEARCHING_SYNC;
volatile uint32_t last_falling_edge = 0;
volatile uint8_t byte_counter = 0;
volatile uint8_t bit_counter = 0;

// The two data bytes your slave wants to write into slots 4 and 5
volatile uint8_t slave_tx_data[2] = {0x3A, 0x0F}; // Example data

FspTimer agt_timer;

void setup() {
Serial.begin(115200);

// Configure Digital Pin 2 (P105) as Input with Pull-up
pinMode(2, INPUT_PULLUP);
// Ensure the output data register is permanently locked to 0
R_PFS->P105PFS_b.PODR = 0;

// Setup AGT0 Timer for 104us intervals
uint8_t timer_type = GPT_TIMER;
int timer_index = 0; // Uses a hardware timer slot
agt_timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_index, 9615.0f, 0.0f, timer_callback);

// Attach external interrupt to Pin 2 on falling edge
attachInterrupt(digitalPinToInterrupt(2, lanc_external_isr, FALLING));
}

void loop() {
// Main application code or telemetry updates
}

// ISR 1: External Falling Edge Trigger (Pin 2)
void lanc_external_isr() {
uint32_t now = micros();

if (lanc_state == SEARCHING_SYNC) {
// Check if the idle period since the last edge matches the >5ms LANC sync gap
if ((now - last_falling_edge) >= SYNC_GAP_MIN) {
lanc_state = PROCESSING_BYTE;
byte_counter = 0;
start_bit_timer(BIT_TIME + HALF_BIT_TIME); // Sample middle of Bit 0 (156us)
}
last_falling_edge = now;
}
else if (lanc_state == WAITING_START) {
// Detected the start bit falling edge of bytes 1 through 7
detachInterrupt(digitalPinToInterrupt(2)); // Block further edge triggers
lanc_state = PROCESSING_BYTE;
start_bit_timer(BIT_TIME + HALF_BIT_TIME); // Sync timer to middle of Bit 0
}
}

// Helper to configure and kick off the hardware timer interval
void start_bit_timer(uint32_t microsecond_delay) {
bit_counter = 0;
agt_timer.setPeriodTime((double)microsecond_delay);
agt_timer.open();
agt_timer.start();
}

// ISR 2: Precise Bit-Sampling/Writing Timer Callback
void timer_callback(timer_callback_args_t *p_args) {
// Reset timer interval back to the standard 104us for subsequent bits
agt_timer.setPeriodTime((double)BIT_TIME);

if (bit_counter < 8) {
// Check if the current frame index is Byte 4 or Byte 5 (Slave Injection Window)
if (byte_counter == 0 || byte_counter == 1) {
uint8_t tx_byte = slave_tx_data[byte_counter];
uint8_t target_bit = (tx_byte >> bit_counter) & 0x01;

if (target_bit == 0) {
R_PFS->P105PFS_b.PDR = 1; // Direct register: Set pin to OUTPUT (Pulls low to write 0)
} else {
R_PFS->P105PFS_b.PDR = 0; // Direct register: Set pin to INPUT (Floats high to write 1)
}
} else {
// Optional: Read bytes 0-3 here if your slave expects specific master commands
// volatile uint8_t bit_val = R_PFS->P105PFS_b.PIDR;
}
bit_counter++;
}
else {
// Reached the Stop Bit / Idle phase of the current byte
R_PFS->P105PFS_b.PDR = 0; // Release the bus immediately (Input mode)
agt_timer.stop();
agt_timer.close();

byte_counter++;

if (byte_counter >= 8) {
// Frame complete, reset state engine to find the next synchronization preamble
lanc_state = SEARCHING_SYNC;
last_falling_edge = micros();
attachInterrupt(digitalPinToInterrupt(2), lanc_external_isr, FALLING);
} else {
// Prepare to capture the start bit of the next sequential byte
lanc_state = WAITING_START;
attachInterrupt(digitalPinToInterrupt(2), lanc_external_isr, FALLING);
}
}
}