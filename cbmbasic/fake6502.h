void reset6502();
void exec6502(uint32_t tickcount);
void step6502();
void irq6502();
void nmi6502();
void hookexternal(void *funcptr);

extern uint16_t pc;
extern uint8_t sp, a, x, y, status;
