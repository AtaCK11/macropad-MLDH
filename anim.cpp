
#include <cmath>
#include <cstring>
#include <cstdlib>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"

#include "anim.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_TAU
#define M_TAU 6.28318530717958647692
#endif

// -------------------- Config / shared state --------------------
namespace {
  // Defaults (can be overridden by anim_config before anim_start_core1)
  int   gW = 128;
  int   gH = 32;
  i2c_inst_t* gI2C = i2c1;
  int   gSDA = 2;
  int   gSCL = 3;
  int   gAddr = 0x3C;
  int   gBaud = 400000;

  // Control flags
  volatile int g_pause = 0;
  volatile int g_frame_ms = 16;
  volatile int g_force_mode = -1;
  volatile int g_next_mode_flag = 0;

  // RNG
  static uint32_t rng_s = 0xA5C3F17u;
  static inline uint32_t rnd(){ rng_s ^= rng_s<<13; rng_s ^= rng_s>>17; rng_s ^= rng_s<<5; return rng_s; }
  static inline int rndr(int a,int b){ return a + (int)(rnd() % (uint32_t)(b-a+1)); }

  // Framebuffer
  // Max size for 128x64: 128 * (64/8) = 1024 bytes
  static uint8_t fb[128 * (64/8)];

  inline void fb_clear() { std::memset(fb, 0, (size_t)gW * (gH/8)); }

  // I2C helpers
  void i2c_send(uint8_t control, const uint8_t* data, size_t n){
    uint8_t buf[17]; buf[0] = control;
    while(n){
      size_t k = n>16?16:n;
      std::memcpy(buf+1, data, k);
      i2c_write_blocking(gI2C, gAddr, buf, k+1, false);
      data += k; n -= k;
    }
  }
  inline void cmd(uint8_t c){ i2c_send(0x00, &c, 1); }
  void ssd1306_init(){
    i2c_init(gI2C, gBaud);
    gpio_set_function(gSDA, GPIO_FUNC_I2C);
    gpio_set_function(gSCL, GPIO_FUNC_I2C);
    gpio_pull_up(gSDA); gpio_pull_up(gSCL);
    sleep_ms(50);
    const uint8_t init[] = {
      0xAE, 0xD5,0x80, 0xA8,(uint8_t)(gH-1), 0xD3,0x00, 0x40,
      0x8D,0x14, 0x20,0x00, 0xA1, 0xC8,
      0xDA,(uint8_t)(gH==64?0x12:0x02), 0x81,
      0x7F, // brightness
      0xD9,0xF1, 0xDB,0x40,
      0xA4, 0xA6, 0x2E, 0xAF
    };
    for (size_t i=0;i<sizeof(init);i++) cmd(init[i]);
  }
  void update_full(){
    cmd(0x21); cmd(0); cmd((uint8_t)(gW-1));
    cmd(0x22); cmd(0); cmd((uint8_t)((gH/8)-1));
    i2c_send(0x40, fb, (size_t)gW * (gH/8));
  }

  // Drawing
  inline void putpix(int x,int y,int on){
    if((unsigned)x >= (unsigned)gW || (unsigned)y >= (unsigned)gH) return;
    int i = x + (y>>3)*gW; uint8_t m = 1u<<(y&7);
    if(on) fb[i]|=m; else fb[i]&=~m;
  }
  void hline(int x0,int x1,int y,int on){
    if(y<0||y>=gH) return; if(x0>x1){int t=x0;x0=x1;x1=t;}
    if(x1<0||x0>=gW) return; if(x0<0)x0=0; if(x1>=gW)x1=gW-1;
    for(int x=x0;x<=x1;x++) putpix(x,y,on);
  }
  void circle(int cx,int cy,int r,int on){
    int x=r,y=0,err=0;
    while(x>=y){
      putpix(cx+x,cy+y,on); putpix(cx+y,cy+x,on);
      putpix(cx-y,cy+x,on); putpix(cx-x,cy+y,on);
      putpix(cx-x,cy-y,on); putpix(cx-y,cy-x,on);
      putpix(cx+y,cy-x,on); putpix(cx+x,cy-y,on);
      y++; if(err<=0){ err+=2*y+1; } else { x--; err-=2*x+1; }
    }
  }
  void fill_circle(int cx,int cy,int r,int on){
    for(int yy=-r; yy<=r; yy++){
      int dx = (int)std::sqrt((float)(r*r - yy*yy));
      hline(cx-dx, cx+dx, cy+yy, on);
    }
  }

  // ---------------- Anim: Solar ----------------
void anim_solar(float &t){
  fb_clear();
  int cx = gW/2, cy = gH/2;
  fill_circle(cx, cy, (gH==64)?5:3, 1);

  int r1=(gH==64)?12:8, r2=(gH==64)?18:12, r3=(gH==64)?24:16;
  for(int a=0;a<360;a+=12){
    float rad = a * (float)M_PI / 180.0f;
    putpix(cx+(int)(r1*cosf(rad)), cy+(int)(r1*sinf(rad)),1);
    putpix(cx+(int)(r2*cosf(rad)), cy+(int)(r2*sinf(rad)),1);
    putpix(cx+(int)(r3*cosf(rad)), cy+(int)(r3*sinf(rad)),1);
  }

  // Wrap phases so arguments stay small and cheap
  float a1 = fmodf(t*0.10f, (float)M_TAU);
  float a2 = fmodf(t*0.06f, (float)M_TAU);
  float a3 = fmodf(t*0.035f,(float)M_TAU);

  int x1=cx+(int)(r1*cosf(a1)), y1=cy+(int)(r1*sinf(a1));
  int x2=cx+(int)(r2*cosf(a2)), y2=cy+(int)(r2*sinf(a2));
  int x3=cx+(int)(r3*cosf(a3)), y3=cy+(int)(r3*sinf(a3));
  fill_circle(x1,y1,2,1); fill_circle(x2,y2,2,1); fill_circle(x3,y3,2,1);

  float m = fmodf(t*0.18f, (float)M_TAU);
  putpix(x2+(int)((gH==64?5:4)*cosf(m)), y2+(int)((gH==64?5:4)*sinf(m)), 1);

  update_full();
}

  // ---------------- Anim: Equalizer ----------------
  constexpr int BARS = 16;
  int eq_h[BARS], eq_target[BARS], eq_peak[BARS];
  void eq_init(){
    for(int i=0;i<BARS;i++){ eq_h[i]=rndr(2,gH-2); eq_target[i]=eq_h[i]; eq_peak[i]=eq_h[i]; }
  }
  void anim_eq(int &frame){
    fb_clear();
    if((frame % 18)==0) for(int i=0;i<BARS;i++) eq_target[i]=rndr(2,gH-2);
    for(int i=0;i<BARS;i++){
      int h=eq_h[i];
      if(h<eq_target[i]) h+=1+(rnd()&1);
      else if(h>eq_target[i]) h-=1+(rnd()&1);
      if(h<1) h=1; if(h>gH-2) h=gH-2; eq_h[i]=h;
      if(eq_peak[i]<h) eq_peak[i]=h; else if((frame % 3)==0 && eq_peak[i]>1) eq_peak[i]--;
    }
    int gap=1, bw=(gW-(BARS-1)*gap)/BARS; if(bw<2) bw=2;
    int x=0;
    for(int i=0;i<BARS;i++){
      int h=eq_h[i];
      for(int yy=gH-1; yy>gH-1-h; yy--) hline(x, x+bw-1, yy, 1);
      hline(x, x+bw-1, gH-1-eq_peak[i], 1);
      x += bw+gap;
    }
    // Border
    hline(0,gW-1,0,1); hline(0,gW-1,gH-1,1);
    for(int y=0;y<gH;y++){ putpix(0,y,1); putpix(gW-1,y,1); }
    update_full();
    frame++;
  }

  // ---------------- Anim: Maze (slow + flash + wait-to-finish) ----------------
  // Tuning
  constexpr int MAZE_MARGIN      = 1;
  constexpr int CELL             = 4;
  constexpr int STEPS_PER_FRAME  = 3;   // corridors per step
  constexpr int FRAME_SKIP       = 2;   // carve every Nth frame

  int MCOLS, MROWS, GRID_W, GRID_H;
  uint8_t* visited = nullptr;

  struct cell_t { int x,y; };
  constexpr int STACK_MAX = 2048;
  cell_t stack_cells[STACK_MAX]; int sp=0;

  inline void push(int x,int y){ if(sp<STACK_MAX) stack_cells[sp++] = {x,y}; }
  inline bool empty(){ return sp==0; }
  inline cell_t top(){ return stack_cells[sp-1]; }
  inline void pop(){ if(sp>0) sp--; }

  void maze_init_dims(){
    GRID_W = gW - MAZE_MARGIN*2;
    GRID_H = gH - MAZE_MARGIN*2;
    MCOLS = GRID_W / CELL; if(MCOLS<2) MCOLS=2;
    MROWS = GRID_H / CELL; if(MROWS<2) MROWS=2;
    int cells = MCOLS*MROWS;
    if(visited) free(visited);
    visited = (uint8_t*)calloc((size_t)cells,1);
  }
  inline void cell_center(int cx,int cy, int &px,int &py){
    px = MAZE_MARGIN + cx*CELL + CELL/2;
    py = MAZE_MARGIN + cy*CELL + CELL/2;
  }
  void carve_between(int x0,int y0,int x1,int y1){
    int px0,py0,px1,py1; cell_center(x0,y0,px0,py0); cell_center(x1,y1,px1,py1);
    // Bresenham
    int dx=std::abs(px1-px0), sx=px0<px1?1:-1;
    int dy=-std::abs(py1-py0), sy=py0<py1?1:-1;
    int err=dx+dy, e2; int x=px0, y=py0;
    for(;;){
      putpix(x,y,1);
      if(x==px1 && y==py1) break;
      e2=2*err; if(e2>=dy){ err+=dy; x+=sx; } if(e2<=dx){ err+=dx; y+=sy; }
    }
  }
  void maze_reset(){
    if(!visited) maze_init_dims();
    std::memset(visited,0,(size_t)MCOLS*MROWS);
    sp=0;
    int sx = rndr(0,MCOLS-1), sy=rndr(0,MROWS-1);
    visited[sy*MCOLS+sx]=1; push(sx,sy);
    fb_clear();
    // border
    for(int x=0;x<gW;x++){ putpix(x,0,1); putpix(x,gH-1,1); }
    for(int y=0;y<gH;y++){ putpix(0,y,1); putpix(gW-1,y,1); }
    update_full();
  }
  inline void maze_flash_solved(){
    for(int i=0;i<4;i++){ cmd(0xA7); update_full(); sleep_ms(120); cmd(0xA6); update_full(); sleep_ms(120); }
  }
  bool maze_step(){
    int steps = 0;
    while(!empty() && steps < STEPS_PER_FRAME){
      cell_t c = top();
      int nlist[4][2]; int ncount=0;
      const int dirs[4][2]={{0,-1},{1,0},{0,1},{-1,0}};
      for(int i=0;i<4;i++){
        int nx=c.x+dirs[i][0], ny=c.y+dirs[i][1];
        if(nx>=0 && nx<MCOLS && ny>=0 && ny<MROWS && !visited[ny*MCOLS+nx]){
          nlist[ncount][0]=nx; nlist[ncount][1]=ny; ncount++;
        }
      }
      if(ncount==0){ pop(); continue; }
      int pick = rndr(0,ncount-1);
      int nx = nlist[pick][0], ny = nlist[pick][1];
      carve_between(c.x,c.y,nx,ny);
      visited[ny*MCOLS+nx]=1;
      push(nx,ny);
      steps++;
    }
    update_full();
    return empty(); // finished
  }

  // --------------- Core1 loop ---------------
  void core1_loop(){
    ssd1306_init();
    eq_init();
    maze_init_dims();

    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    int mode = 0;            // 0=Solar, 1=EQ, 2=Maze
    float t = 0.0f;
    int eq_frame = 0;
    uint32_t maze_frame = 0;
    bool maze_done = false;
    bool entered_maze = false;

    while (1) {
      if(!g_pause){
        uint32_t now = to_ms_since_boot(get_absolute_time());
        switch(mode){
          case 0: anim_solar(t); break;
          case 1: anim_eq(eq_frame); break;
          case 2: {
            // slow down carving via frame skip
            maze_frame++;
            if ((maze_frame % FRAME_SKIP) == 0) {
              if(maze_step()){ maze_flash_solved(); maze_done = true; }
            }
            if(maze_done){ mode = (mode+1)%3; t0 = now; entered_maze=false; }
            break;
          }
        }
        t += 0.06f;
        if (t > 100000.0f) t = fmodf(t, (float)M_TAU);
      }
      sleep_ms(g_frame_ms);
      tight_loop_contents();
    }
  }

  // Trampoline for core1 (C ABI)
  void core1_entry_trampoline(){
    core1_loop();
  }
} // namespace

// -------------------- Public C API --------------------
extern "C" {

void anim_config(int width_px, int height_px, void* i2c_inst, int sda_pin, int scl_pin, int i2c_address, int i2c_baud){
  if(width_px==128 || width_px==64) gW = width_px; else gW = 128;
  gH = (height_px==64 ? 64 : 32);
  gI2C = (i2c_inst_t*)i2c_inst;
  gSDA = sda_pin;
  gSCL = scl_pin;
  gAddr = i2c_address;
  gBaud = i2c_baud;
}

void anim_start_core1(void){
  static bool started = false;
  if(started) return;
  started = true;
  multicore_launch_core1(core1_entry_trampoline);
}

void anim_next_mode(void){ g_next_mode_flag = 1; }

void anim_set_mode(anim_mode_t m){
  if(m < ANIM_MODE_SOLAR || m > ANIM_MODE_MAZE) return;
  g_force_mode = (int)m;
}

void anim_pause(int paused){ g_pause = paused ? 1 : 0; }

void anim_set_speed(int frame_ms){ if(frame_ms < 1) frame_ms = 1; g_frame_ms = frame_ms; }

void anim_set_brightness(uint8_t level) {
  if (level > 255) level = 255;
  cmd(0x81);
  cmd(level);
}

} // extern "C"
