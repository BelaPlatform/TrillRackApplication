// this is only needed when TrillRackApplicationStm32.cpp drops some defines.
// As this contains some big-ish static objects, we don't want to build it in regular use,
//although the linker may be smart enough to know it doesn't need to link it in. But as I see some small ( some 200 bytes) in binary size when this is ifdef'ed out, it's probably best to exclude it
#if 0
#include <TrillRackApplication_bsp.h>
#include <trill-neopixel/Stm32NeoPixel.h>
extern "C" {
int npSetup(void);
ssize_t npSend(const uint8_t* rgb, size_t length);
void npSendColorToAll(uint32_t val);
void npDone(void);
uint8_t npReady(void);

int trillSetup();
void trillNewData(const uint8_t* data, size_t len);
int trillRead();
unsigned int trillNumTouches();
float trillTouchLocation(unsigned int n);
float trillTouchSize(unsigned int n);
}

enum { kNumLeds = 23 };
static Stm32NeoPixelT<uint32_t, kNumLeds> stm32Np(&neoPixelHtim, neoPixelHtim_TIM_CHANNEL_x, 0.66 * neoPixelHtim_COUNTER_PERIOD, 0.33 * neoPixelHtim_COUNTER_PERIOD);
static NeoPixelT<kNumLeds> np;

int npSetup()
{
  np.setSnp(&stm32Np);
  npSendColorToAll(0);
  return 0;
}

ssize_t npSend(const uint8_t* rgb, size_t length)
{
  for(unsigned int n = 0; n < length / 3 && n < kNumLeds; ++n)
  {
    np.setPixelColor(n, rgb[n * 3 + 0], rgb[n * 3 + 1], rgb[n * 3 + 2]);
  }
  np.show();
  return length;
}

void npDone()
{
  stm32Np.done();
}

uint8_t npReady()
{
  return stm32Np.ready();
}

void npSendColorToAll(uint32_t val)
{
  uint8_t toSend[kNumLeds * 3];
  for(unsigned int n = 0; n < sizeof(toSend); ++n)
  {
      uint8_t byte = (val >> ((8 * (n % 3)) & 0xff));
      toSend[n] = byte;
  };
  npSend(toSend, sizeof(toSend));
}

#ifdef HEX
#undef HEX
#endif
#include <libraries/Trill/Trill.h>
static Trill trill;

int trillSetup()
{
  if(trill.setup(1, Trill::BAR, 0x20))
    return -1;
  trill.printDetails();
  if(trill.setMode(Trill::CENTROID))
    return -2;
  if(trill.setPrescaler(2))
    return -3;
  if(trill.updateBaseline())
    return -4;
  if(trill.prepareForDataRead())
    return -5;
  return 0;

  //cd.setup({padsToOrderMap, padsToOrderMap + kNumPads / 2}, 4, 3200);

  //trill.setMode(Trill::DIFF);
}

void trillNewData(const uint8_t* data, size_t len)
{
  trill.newData(data, len);
}

int trillRead()
{
  return trill.readI2C();
}

unsigned int trillNumTouches()
{
  return trill.getNumTouches();
}

float trillTouchLocation(unsigned int n)
{
  return trill.touchLocation(n);
}

float trillTouchSize(unsigned int n)
{
  return trill.touchSize(n);
}
#endif
