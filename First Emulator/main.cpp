#include <cmath>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <map>
#include <chrono>
#define VECT 0x400C
#include <SDL3/SDL.h>
// #define WHOLEDRAW


struct Processor {
    uint8_t address[1 << 16];
    uint16_t ip = 0;
    uint8_t sp = 0;
    uint8_t a = 0;
    uint8_t s = 0;
    uint8_t x = 0;
    uint8_t y = 0;
    int cycle = 0;
    const float clock = 1.0;
};

struct Bus {
    uint8_t read(uint16_t offset);
    void write(uint16_t offset, uint8_t data);
};

struct Modulator {
    const float clock = 21.47;
};

struct Display {
    int line = 0;
    int lineHor = 0;
    int lineVer = 0;
    int dot = 0;
    bool blanking = false; 
    const float clock = 5.0;
    int cycle = 0;
    const double drawTime = 60.0;
    const double blankingTime = 20.0;
    const double frameTime = drawTime + blankingTime;
};

struct Opcode {
    uint8_t op;
    uint8_t addressing;
    uint8_t size;
    uint8_t cycles;
};  

Bus *bus{};
Opcode *op;
Opcode arr_ptr[sizeof(op)];
Processor proc{};
Processor graph{};
Display display{};
Modulator *mod{};

SDL_Window *window = SDL_CreateWindow("testetest", 1000, 1000, 0);
SDL_Event e;
SDL_Window *memory = SDL_CreateWindow("memview", 400, 800, 0);
SDL_Renderer *renderer = SDL_CreateRenderer(window, NULL);

int phx() {
    proc.address[proc.sp] = proc.x;
    proc.sp++;
}

int plx() {
    proc.sp--;
    proc.x = proc.address[proc.sp];
}

int inx() {
    proc.x++;
}

int lsr() {
    proc.a <<= 1;
    return 1;
}

int asl() {
    proc.s = proc.s & 0xf8;
    proc.s = proc.s | proc.a & 0x80 >> 6;
    proc.a = proc.a << 1;
    return proc.a;
}

int phy() {
    proc.address[proc.sp] = proc.x;
    proc.sp++;
}

// int lda_abs() {
//     proc.ip++;
//     proc.a = bus->read(proc.ip);
// }

int sta_abs() {
    // proc.ip++; 
    // uint8_t address = bus->read(proc.ip);
    bus->write(bus->read(proc.ip+1), proc.a);
}

int sta_zp() {
    bus->write(bus->read(proc.ip+1), proc.a);
}

int txs() {
    proc.sp = proc.x;
}

int tsx() {
    proc.x = proc.sp;
}

int txa() {
    proc.a = proc.x;
}

int cmp() {

    proc.s &= 0x3E;
    uint8_t value = proc.a - proc.address[proc.ip+1]; // << 7;
    if (!value) {
        proc.s | 0x40;
    }
    else {
        proc.s | 0x80;
        proc.s | value >> 8;
    }
}

int cpx() {
    proc.s &= 0x3E;
    uint8_t value = proc.x - proc.address[proc.ip+1]; // << 7;
    if (!value) {
        proc.s | 0x40;
    }
    else {
        proc.s | 0x80;
        proc.s | value >> 8;
    }
}

int cpy() {
    proc.s &= 0x3E;
    uint8_t value = proc.y - proc.address[proc.ip+1]; // << 7;
    if (!value) {
        proc.s | 0x40;
    }
    else {
        proc.s | 0x80;
        proc.s | value >> 8;
    }
}

int bne() {
    if(!proc.s & 0x40) {
        proc.ip += abs(proc.address[proc.ip+1]);
    }
}

int beq() {
    if(proc.s & 0x40) {
        proc.ip += abs(proc.address[proc.ip+1]);
    }   
}

int bcc() {
    if (!proc.s & 0x80) {
        proc.ip += abs(proc.address[proc.ip+1]);
    }
}

int bcs() {
    if (proc.s & 0x80) {
        proc.ip += abs(proc.address[proc.ip+1]);
    }
}

int rts() {
    proc.ip = (proc.address[proc.sp-1] << 8 + proc.address[proc.sp-2]);
    proc.sp -= 2;
}

int ora() {
    proc.a | proc.address[proc.ip+1];
}

int bmi() {
    if(proc.s & 0x01) {
        proc.ip += abs(proc.address[proc.ip+1]);
    }
}

int bpl() {
    if(!proc.s & 0x01) {
        proc.ip += abs(proc.address[proc.ip+1]);
    }
}

int tax() {
    proc.x = proc.a;
}

int tya() {
    proc.a = proc.y;
}

int tay() {
    proc.y = proc.a;
}

int clc() {
    proc.s &= 0x7F;
}

int sec() {
    proc.s | 0x80; 
}

int rol() {
    proc.a = proc.a << 1;
    //
}

int ror() {
    proc.a = proc.a >> 1;
    //
}

int eor() {
    proc.a ^= proc.address[proc.ip+1];
}

int php() {
    proc.sp++;
    proc.address[proc.sp] = proc.s; 
}

int plp() {
    proc.s = proc.address[proc.sp];
    proc.sp--;
}

int ply() {
    proc.y = proc.address[proc.sp];
    proc.sp--;
}

int jmp() {
    proc.ip = proc.address[proc.ip+1] << 8 + proc.address[proc.ip+2];
}

int cli() {
    proc.s ^= 0x04; 
}

int stx_abs() {
    proc.address[proc.ip + 1 << 8 + proc.ip + 2] = proc.x;
}

int sty_abs() {
    proc.address[proc.ip + 1 << 8 + proc.ip + 2] = proc.y;
}

int lda_abs() {
    uint16_t address = bus->read(proc.ip + 1) << 8 + bus->read(proc.ip + 2);
    proc.a = bus->read(address);
}

int lda_imm() {
    proc.a = bus->read(proc.ip + 1);
}

int lda_abs_y() {
    uint16_t address = bus->read(proc.ip + 1) << 8 + bus->read(proc.ip + 2);
    uint16_t pointer = bus->read(address + proc.y);
    proc.a = pointer;
}

int lda_abs_x() {
    uint16_t address = bus->read(proc.ip + 1) << 8 + bus->read(proc.ip + 2);
    address += proc.x * 2;
    uint16_t pointer = bus->read(address);
}

int lda_zp() {
    uint8_t address = proc.address[proc.ip + 1];
    proc.a = proc.address[address];
}

int lda_indirectx() {
    uint16_t address = proc.address[proc.ip + 1] << 8 + proc.address[proc.ip + 2];
    proc.a = proc.address[address + proc.x];
}

int lda_indirecty() {
    uint16_t address = proc.address[proc.ip + 1] << 8 + proc.address[proc.ip + 2];
    address = proc.address[address];
    proc.a = proc.address[address + proc.y];
}

int jsr() {
    uint16_t address = proc.address[proc.ip + 1] << 8 + proc.address[proc.ip + 2];
    proc.ip = address;
}

int ldx() {

}

int ldy() {

}

int stx() {

}

int sty() {

}

int iny() {

}

int dex() {

}

int dey() {

}

int andt() {
    
}

int (*arr_op[])() = { asl, lsr, phx, phy, plx}; //, jsr, lda, ldx, ldy, rts, php, plp, plx, ply, sta, sta_zp, sta_abs, adc, sbc, clc, sec, rol, ror, stx, sty, beq, bne, bpl, bmi, bvs, bvc, bcc, bcs, inc, dec, inx, iny, dex, dey, and, ora, eor, cmp, cpx, cpy, tax, tya, txa, tay, txs, tsx, cli, sei, jmp };

bool scrollRead = 0;
bool vramRead = 0;

bool OAMDMA = 0;
bool NMI_STATUS = 0;

uint8_t scroll_Hor = 0;
uint8_t scroll_Ver = 0;
uint8_t *scroll = &scroll_Hor;
uint8_t vram_low = 0;
uint8_t vram_high = 0;
uint16_t VRAM = 0;
uint8_t *vram_offset = &vram_low;

void DMATransfer(int offset, int size, Processor *source, Processor *dest) {
    for(int it = 0; it < size; it++)
        dest->address[offset] = source->address[offset];
}

void updateRegisters() {
    uint8_t object_0_Hor = graph.address[0x200];
    uint8_t object_0_Ver = graph.address[0x204];
    if (proc.address[0x2002] | 0x40 == 0x40) {
        proc.address[0x2002] ^= 0x40;
    }
    // uint8_t pattern =     proc.address[(object_0_Ver*256+object_0_Ver)/8];
    if (object_0_Ver == display.line && object_0_Hor == display.line) {
        proc.address[0x2002] = proc.address[0x2002] | 0x40;
    }
    if (!scrollRead)
        scroll_Hor = proc.address[0x2005];
    else 
        scroll_Ver = proc.address[0x2005];
    if (!vramRead)
        vram_low = proc.address[0x2006];
    else
        vram_high = proc.address[0x2006];
    // scroll[scrollRead] = proc.address[0x2005];
    // scrollRead = !scrollRead;
    // vram_offset[vramRead] = proc.address[0x2006];
    // vramRead = !vramRead;
    (OAMDMA ? 0 : 0);
}

void render(uint8_t data[], int x, int y, int size) {
    const int SPRITE_WIDTH = 8;
    const int SCREEN_WIDTH = 256;
    const int SCREEN_HEIGHT = 240;
    const int SPRITE_HEIGHT = 8;
    const int SCREEN_SIZE = SCREEN_WIDTH * SCREEN_HEIGHT;
    const int OM_SIZE = 0xff;
    int screen = 0x2200;
    int attribute = 0x3F00;
    int Rgb[64] = {};
    int secondaryObjMemory[64];

    #ifdef WHOLEDRAW 
    uint8_t offsetHor = scroll_Hor % SPRITE_WIDTH;
    uint8_t offsetVer = scroll_Hor % SPRITE_HEIGHT;
    for(int i = 0; i < SCREEN_SIZE; i++) {
        int x = 0; 
        int y = 0;
        uint8_t pattern = graph.address[0x2000+i];
        pattern = graph.address[pattern*64];
        for(int j = 0; j < 8*8; j++) {
            int px = 0; 
            int py = 0; 
            uint8_t buffer = graph.address[pattern+(py*8+px)+(!!offsetVer*(offsetVer+py)*8)+(!!offsetHor*(offsetHor+px))];     
            // render
            
            px++;
            if(px > 8) {
                px = 0;
                py++;
                // if(py > 8) {
                //     break;
                // }
            }
        }
        x += 8;
        if(x > SCREEN_WIDTH) {
            x = 0;
            y++;
            // if(y > SCREEN_HEIGHT) {
            //     break;
            // }
        }
    }

    #else
    for(int i = 0; i < size; i++) {
        uint8_t pattern = graph.address[0x2000 + ((display.lineVer - size + i / 8) * SCREEN_WIDTH + (display.lineHor - size + i / 8))];
        pattern = graph.address[pattern*64];
        uint8_t offsetHor = (display.lineHor - size + i) % 8 + (scroll_Hor % 8); 
        uint8_t offsetVer = (display.lineVer - size + i) % 8 + (scroll_Ver % 8);
        // int x = display.lineHor;
        // int y = display.lineVer;
        // int px = 0;
        // int py = 0;
        uint8_t _a = graph.address[pattern + offsetVer] & offsetHor >> offsetHor;
        uint8_t _b = graph.address[pattern + 8 + offsetVer] & offsetHor >> offsetHor;
        uint8_t offset = _a + _b;
        uint8_t attribute = graph.address[screen + (offsetVer * 8) + offsetHor];
        uint8_t value = graph.address[0x3F00 + (attribute * 4) + offset] * 3;
        uint8_t r = Rgb[value+0];
        uint8_t g = Rgb[value+1];
        uint8_t b = Rgb[value+2];
        // uint8_t buffer = graph.address[pattern+(py*8+px)+(!!offsetVer*(offsetVer+py)*8)+(!!offsetHor*(offsetHor+px))];
    }
    #endif
    
    for(int i = 0; i < size; i += 4) {
        if(display.lineHor == 0 && display.lineVer == 0) {
            int i, j;
            for (i = 0x0200, j = 0; i < 256 || j < (8 * 8); i += 4) {
                if (graph.address[i] == display.lineVer) {
                    for (int k = i; k < 4; k++) {
                        secondaryObjMemory[j] = graph.address[k];
                        j += 1;
                    }
                }
            }
        }
    }

    for(int i = 0; i < size; i += 4) {
        uint8_t offsetHor = x - size + i & 0x07; // + (scroll_Hor & 0x07); 
        uint8_t offsetVer = y - size + i & 0x07; // + (scroll_Ver & 0x07);
        uint8_t y = secondaryObjMemory[i];
        if (y > SCREEN_HEIGHT) {
            continue;
        }
        uint8_t pattern = secondaryObjMemory[i];
        uint8_t attributes = secondaryObjMemory[i];
        uint8_t x = secondaryObjMemory[i];
        uint8_t p = attributes & 0x03;
        pattern = graph.address[pattern * 64];

        uint8_t _a = graph.address[pattern] & (0x80 >> offsetHor) >> 8 - offsetHor;
        uint8_t _b = graph.address[pattern+8] & (0x80 >> offsetHor) >> 8 - offsetHor;
        uint8_t buffer = _a + _b;
        uint8_t value = graph.address[0x3F00 + (p * 4) + buffer] * 3;
        uint8_t r = Rgb[value+0];
        uint8_t g = Rgb[value+1];
        uint8_t b = Rgb[value+2];
        SDL_SetRenderDrawColor(renderer, r, g, b, 255);
        SDL_RenderPoint(renderer, x, y);
    }

    // uint8_t attribute = graph.address[screen + (offsetVer * 8) + offsetHor];
    
    #ifdef WHOLEDRAW
    for(int i = 0; i < OM_SIZE; i += 4) {
        uint8_t y = graph.address[0x200+i];
        if(y > SCREEN_HEIGHT) {
            continue;
        }
        uint8_t pattern = graph.address[0x201+i];
        uint8_t param = graph.address[0x202+i];
        uint8_t x = graph.address[0x203+i];
        uint8_t p = param & 0x03;
        for(int j = 0; j < 8*8; j++) {
            int px = 0;
            int py = 0;
            uint8_t buffer = graph.address[pattern*64+(py*8+px)];
            // render
            if(px > 8) {
                px = 0;
                py++;
                // if(y > 8) {
                //     break;
                // }
            }
        }
    }
    #endif
}

uint8_t getOpcode(uint16_t ip) {
    return proc.address[ip];
}

int execOpcode(uint8_t op) {
    return arr_op[op]();
}

uint8_t CONTROL_1;
uint8_t CONTROL_2;
uint8_t cread;
uint8_t Rgb[64*3];

int main(int argc, char* argv[]) {
    bool *blank = &display.blanking;
    int *line = &display.line;
    *blank = false;
        
    SDL_Init(SDL_INIT_VIDEO);

    std::fstream file;
    file.open("testrom2.nes", std::ios::in | 
        std::ios::binary);
    uint8_t data[65535];
    file.read(reinterpret_cast<char*>(&data), sizeof(data));
    // for (int i = 0; i < sizeof(data); i++)
    //     std::cout << data[i] << std::endl;
    // std::ifstream file;
    // std::string data;
    // file >> data;
    // uint8_t offset = 4;
    // const uint8_t header[3] = { *"N", *"E", *"S"};
    // for (uint8_t i = 0; i < 3; i++) {
    //     try {
    //         if (header[i] != data[i]) {
    //             throw(header);
    //         }
    //     }
    //     catch (int header) {
    //         std::cout << header;
    //     }
    // }
    const uint8_t PRG_ROM_SIZE = data[4];
    const uint8_t CHR_ROM_SIZE = data[5];
    const uint8_t MAPPER_PRMTA = data[6];
    const uint8_t MAPPER_PRMTB = data[7];
    const uint8_t PRG_RAM_SIZE = data[8];
    for (uint16_t i = 16; i < (1 << 14) * PRG_ROM_SIZE; i++) {
        proc.address[0xC000 + i - 16] = data[i];
    }
    for (uint16_t i = 0; i < (1 << 13) * CHR_ROM_SIZE; i++) {
        graph.address[i] = data[PRG_ROM_SIZE + i];
    }
    proc.ip = graph.address[(1 << 14) * PRG_ROM_SIZE - 3] << 8 + graph.address[1 << 14 * PRG_ROM_SIZE - 4];

    uint32_t frame = 0;
    double time = 0.0;
    int clockCycleSync = ceil((21.47/4) / (21.47/12));
    const double frametime = 16.67;
    int cycles = 0;

    const int SCREEN_WIDTH = 256;
    const int SCREEN_HEIGHT = 240;
    const int BLANK_TIME = 20;

    std::map<SDL_Keycode, bool> keyState;
    
    std::chrono::time_point<std::chrono::system_clock> Time();
    
    bool open = true;
    
    while(open) { 
        
        if(display.line % 3 == 0) { 
            uint8_t op = getOpcode(proc.ip);
            cycles = execOpcode(op);
            display.lineHor = display.cycle += cycles * 3;
            // display.line++;
        }
        else {
            display.cycle = display.lineHor++;
            cycles = 1;
        }
        if (display.lineHor > SCREEN_WIDTH) {
            display.lineHor -= SCREEN_WIDTH;
            display.lineVer += 1;
        }
        display.line = display.lineHor + display.lineVer;
        updateRegisters();
        if (display.lineVer < SCREEN_HEIGHT) { // (display.line <= 256 * 240) {    
            render(graph.address, display.lineHor, display.lineVer, cycles);
        }
        while(time <= std::fmod(time, (double)16.67) && display.line >= SCREEN_WIDTH*SCREEN_HEIGHT && !*blank) {
            time++;
        }
        if(time >= std::fmod(time, (double)16.67) && display.line >= SCREEN_WIDTH*SCREEN_HEIGHT && !*blank) {    
            proc.s = proc.s | 0x20;
            *blank = !*blank;
            bus->write(0x2002, bus->read(0x2000) | 0x80); // = bus->read(0x2000) | 0x80;
            if(bus->read(0x2000) & 0x80 >> 8) {
                proc.address[proc.sp] = proc.s;
                proc.address[proc.sp + 1] = proc.ip;
                proc.sp += 2;
                proc.ip = proc.address[(1 << 14) * PRG_ROM_SIZE - 5] << 8 + proc.address[(1 << 14) * PRG_ROM_SIZE - 6];
            }   
        } 
        while(time <= std::fmod(time, (double)1.38) && display.line >= SCREEN_WIDTH*(SCREEN_HEIGHT+BLANK_TIME) && *blank) {
            time++;
        }

        if(time == std::fmod(time, (double)1.38) && *blank || display.line == SCREEN_WIDTH*(SCREEN_HEIGHT+BLANK_TIME) && *blank) {
            proc.s = proc.s & 0xDF;
            display.line = !display.line;
            display.lineHor = display.lineVer = 0;
            *blank = !*blank;
            frame++;
        }
        // std::chrono::time_point<std::chrono::system_clock> _time = Time();
        time++;
    }
}

uint8_t Bus::read(uint16_t offset) {
    uint8_t value;
    switch (offset) {
        case 0x2002:
            display.blanking = false;
            break;
        case 0x2007:
            return graph.address[vram_low << 8 + vram_high];
            break;
        case 0x4016:
            (cread == 0x80 ? 0 : cread);
            value = (CONTROL_1 & cread) >> cread % 8;
            cread *= 2;
            return value; 
        case 0x4017:
            break; 
        default: 
        break;
    }
    return proc.address[offset]; 
}

int SCREENSET;
int VPARAM;
int INTST;
int LATCH_1;
int LATCH_2;

void Bus::write(uint16_t offset, uint8_t data) {

    if(offset >= 0x8000 | offset < 0x0000) {
        return;
    }

    if(offset >= 0x0800 && offset <= 0x3FFF) {
        offset = offset & 0x07FF;
    }

    switch (offset) {
    case 0x2000:
        SCREENSET = data & 0x03;
        VPARAM = data & 0x04 >> 2;
        NMI_STATUS = data & 0x80 >> 8;
        break;
    case 0x2001:
        break;
    case 0x2003:
        break;
    case 0x2004:
        break;
    case 0x2005:
        scrollRead = !scrollRead;
        break;
    case 0x2006: 
        if(vramRead) {
            vram_low = data;
        }
        else {
            vram_high = data;
        }
        vramRead = !vramRead; 
        VRAM = vram_high << 8 + vram_low;
        break;
    case 0x2007:
        graph.address[VRAM] = data; 
        if (VPARAM) {
            VRAM += 1; 
        }
        else {
            VRAM += 32;
        }
    case 0x4014:
        DMATransfer(0x200, 0xff, &proc, &graph);
        break;
    case 0x4016:
        LATCH_1 = true;
    case 0x4017:
        LATCH_2 = true;
    default:
        break;
    }
    proc.address[offset] = data;
}







void reproc(Processor *pr) {
    
}

void redisp(Display *d) {
    d->line = 0;
    d->cycle = 0; 
    d->blanking = false;
}

