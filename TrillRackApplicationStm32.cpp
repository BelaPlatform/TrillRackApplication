#include "TrillRackApplicationStm32.h"
#include <TrillRackApplication_bsp.h>
#include <main.h>
#include "retarget.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

enum {
  kCommandNone = 0,
  kCommandMode = 1,
  kCommandScanSettings = 2,
  kCommandPrescaler = 3,
  kCommandNoiseThreshold = 4,
  kCommandIdac = 5,
  kCommandBaselineUpdate = 6,
  kCommandMinimumSize = 7,
  kCommandAdjacentCentroidNoiseThreshold = 8,
  kCommandAutoScanInterval = 16,
  kCommandIdentify = 255
};

enum {
    kModeCentroid = 0,
    kModeRaw = 1,
    kModeBaseline = 2,
    kModeDiff = 3
};

enum {
  kOffsetCommand = 0,
  kOffsetData = 4
};

enum { kNumTouches = 5 };
// The device 7 bits address value in datasheet must be shifted to the left before calling the interface
// may be overridden below
uint8_t gI2cAddress = 0x20 << 1;
const unsigned int kTimeout = 10;

#define TOGGLE_DEBUG_PINS
#define I2C_USE_DMA
#define DAC_USE_DMA
#define ADC_USE_DMA
#define GPIO_OUT_USE_DMA
#define GPIO_IN_USE_DMA
#define NEOPIXEL_USE_TIM
#define TRILL_RACK_INTERFACE

#ifdef TRILL_RACK_INTERFACE
#include "trill-neopixel/TrillRackInterface.h"
#endif // TRILL_RACK_INTERFACE

#ifdef I2C_USE_DMA
#define TRILL_USE_CLASS

#ifdef TRILL_USE_CLASS
int trillSetup();
void trillNewData(const uint8_t* data, size_t len);
int trillRead();
unsigned int trillNumTouches();
float trillTouchLocation(unsigned int n);
float trillTouchSize(unsigned int n);
#endif // TRILL_USE_CLASS

enum { gI2cDmaRecvSize = 48 }; //kNumTouches * 2 * 2 };
uint8_t gI2cDmaRecv[gI2cDmaRecvSize];
uint8_t gI2cLatestRecv[gI2cDmaRecvSize];

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  fprintf(stderr, "HAL_I2C_ErrorCallback. TODO: handle me\n\r");
}

// In order for this to be called, the I2C1 EVENT interrupt must be manually enabled in CubeMx.
// See https://blog.shirtec.com/2019/10/stm32-hal-i2c-itdma-gotcha.html
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
//  for(unsigned int n = 0; n < gI2cDmaRecvSize; n += 2)
//    printf("%d ", gI2cDmaRecv[n] * 256 + gI2cDmaRecv[n + 1]);
//  printf("\n\r");
#ifdef TRILL_RACK_INTERFACE
  tr_newData(gI2cDmaRecv, gI2cDmaRecvSize);
#else // TRILL_RACK_INTERFACE
#ifdef TRILL_USE_CLASS
  trillNewData(gI2cDmaRecv, gI2cDmaRecvSize);
#else // TRILL_USE_CLASS
  memcpy(gI2cLatestRecv, gI2cDmaRecv, gI2cDmaRecvSize);
#endif // TRILL_USE_CLASS
#endif // TRILL_RACK_INTERFACE
  HAL_I2C_Master_Receive_DMA(&trillHi2c, gI2cAddress, gI2cDmaRecv, gI2cDmaRecvSize);
}
#endif // I2C_USE_DMA

typedef enum {
#ifdef DAC_USE_DMA
  kDAC0,
  kDAC1,
#endif //DAC_USE_DMA
#ifdef ADC_USE_DMA
  kADC,
#endif // ADC_USE_DMA
#ifdef GPIO_OUT_USE_DMA
  kGpioOut,
#endif // GPIO_OUT_USE_DMA
#ifdef GPIO_IN_USE_DMA
  kGpioIn,
#endif // GPIO_IN_USE_DMA
  kNumStreams,
} Stream;

static void streamComplete(Stream stream, uint8_t end);
enum { kDoubleBufferSize = 64 };

#ifdef DAC_USE_DMA
enum { kDacNumChannels = 2 };
static uint16_t gDacOutputs[kDacNumChannels][kDoubleBufferSize];
uint16_t gDacNext[kDacNumChannels];

static void dacCb(unsigned int channel, unsigned int end)
{
#ifdef TOGGLE_DEBUG_PINS
  HAL_GPIO_WritePin(DEBUG0_GPIO_Port, DEBUG0_Pin, GPIO_PIN_SET);
  if(0 == end)
    HAL_GPIO_WritePin(DEBUG0_GPIO_Port, DEBUG0_Pin, GPIO_PIN_RESET);
#endif // TOGGLE_DEBUG_PINS
  Stream stream = channel ? kDAC1 : kDAC0;
  streamComplete(stream, end);
#ifdef TOGGLE_DEBUG_PINS
  if(1 == end)
    HAL_GPIO_WritePin(DEBUG0_GPIO_Port, DEBUG0_Pin, GPIO_PIN_RESET);
#endif // TOGGLE_DEBUG_PINS
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  dacCb(0, 0);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  dacCb(0, 1);
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  dacCb(1, 0);
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  dacCb(1, 1);
}

static void dacError(unsigned int dac, unsigned int dma)
{
  fprintf(stderr, "DAC error: %d %d\n\r", dac, dma);
}

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac)
{
  dacError(0, 0);
}

void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac)
{
  dacError(0, 1);
}

void HAL_DAC_ErrorCallbackCh2(DAC_HandleTypeDef *hdac)
{
  dacError(1, 0);
}

void HAL_DAC_DMAUnderrunCallbackCh2(DAC_HandleTypeDef *hdac)
{
  dacError(1, 1);
}

#endif // DAC_USE_DMA
#ifdef ADC_USE_DMA
uint16_t gAdcInputs[kDoubleBufferSize];

static void adcCb(unsigned int end)
{
#ifdef TOGGLE_DEBUG_PINS
  HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_RESET);
#endif // TOGGLE_DEBUG_PINS
  streamComplete(kADC, end);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  adcCb(0);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  adcCb(1);
}
#endif // ADC_USE_DMA

#ifdef GPIO_OUT_USE_DMA
static uint32_t gGpioOut[kDoubleBufferSize];
GPIO_TypeDef* gGpioHighRateOutBank = GPIOB;
const unsigned int kGpioHighRateOutTimerChannel = gpioHtimChannelOut;
const uint16_t kGpioOutMask = 0 & (1 << 6); // what GPIO bits to actually write to

static void digitalWriteInit(uint8_t end)
{
  unsigned int off = end ? kDoubleBufferSize / 2 : 0;
  memset(gGpioOut + off, 0, sizeof(gGpioOut) / 2);
}

static void digitalWrite(uint8_t end, unsigned int frame, unsigned int channel, uint8_t val)
{
  unsigned int off = end ? kDoubleBufferSize / 2 : 0;
  // the word will be written to GPIOx_BSRR Bit Set Reset register
  // if(val), set bit in the lower half-word(SET), else in the upper half-word(RESET)
  gGpioOut[frame + off] |= 1 << (channel + (!val) * 16) & (kGpioOutMask | (kGpioOutMask << 16));
}

static void gpioHighRateOutDone(uint8_t end)
{
#ifdef TOGGLE_DEBUG_PINS
  HAL_GPIO_WritePin(DEBUG2_GPIO_Port, DEBUG2_Pin, GPIO_PIN_SET);
#endif // TOGGLE_DEBUG_PINS
  digitalWriteInit(end);
  streamComplete(kGpioOut, end);
#ifdef TOGGLE_DEBUG_PINS
  HAL_GPIO_WritePin(DEBUG2_GPIO_Port, DEBUG2_Pin, GPIO_PIN_RESET);
#endif // TOGGLE_DEBUG_PINS
}

static void gpioHighRateOutCpltCb(DMA_HandleTypeDef* hdma)
{
  gpioHighRateOutDone(1);
}

static void gpioHighRateOutHalfCpltCb(DMA_HandleTypeDef* hdma)
{
  gpioHighRateOutDone(0);
}

static void gpioErrorCb(DMA_HandleTypeDef* hdma)
{
  fprintf(stderr, "GPIO error\n\r");
}
#endif // GPIO_OUT_USE_DMA

#ifdef GPIO_IN_USE_DMA

static uint16_t gGpioIn[kDoubleBufferSize];
const GPIO_TypeDef* gGpioHighRateInBank = GPIOB;
const unsigned int kGpioHighRateInTimerChannel = gpioHtimChannelIn;

static inline int digitalRead(uint8_t end, unsigned int frame, unsigned int channel)
{
  unsigned int off = end ? kDoubleBufferSize / 2 : 0;
  uint16_t val = gGpioIn[frame + off] & (1 << (channel));
  // make it 1 or 0
  return !(!val);
}

static void gpioHighRateInDone(uint8_t end)
{
#ifdef TOGGLE_DEBUG_PINS
//  HAL_GPIO_WritePin(DEBUG3_GPIO_Port, DEBUG3_Pin, 1);
#endif // TOGGLE_DEBUG_PINS
  streamComplete(kGpioIn, end);
//  printf("%x \n\r", digitalRead(end, 0, 4));
#ifdef TOGGLE_DEBUG_PINS
//  HAL_GPIO_WritePin(DEBUG3_GPIO_Port, DEBUG3_Pin, 0);
#endif // TOGGLE_DEBUG_PINS
}

static void gpioHighRateInCpltCb(DMA_HandleTypeDef* hdma)
{
  gpioHighRateInDone(1);
}

static void gpioHighRateInHalfCpltCb(DMA_HandleTypeDef* hdma)
{
  gpioHighRateInDone(0);
}
#endif // GPIO_IN_USE_DMA

#if defined(GPIO_OUT_USE_DMA) || defined(GPIO_IN_USE_DMA)

static uint32_t getTimChannel(unsigned int channel)
{
  uint32_t vals[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_5, TIM_CHANNEL_6, TIM_CHANNEL_ALL};
  return vals[channel];
}

static uint32_t getTimDmaId(unsigned int channel)
{
  uint32_t vals[] = {TIM_DMA_ID_CC1, TIM_DMA_ID_CC2, TIM_DMA_ID_CC3, TIM_DMA_ID_CC4};
  return vals[channel - 1];
}

static uint32_t getTimDmaRequestEnableBit(unsigned int channel)
{
  uint32_t vals[] = {TIM_DMA_CC1, TIM_DMA_CC2, TIM_DMA_CC3, TIM_DMA_CC4};
  return vals[channel - 1];
}

static int gpioHighRateInitDma(unsigned int timerChannel, const volatile uint32_t * source, volatile uint32_t* dest,
    void (*halfCplt)(DMA_HandleTypeDef*), void (*cplt)(DMA_HandleTypeDef*))
{
  uint32_t TIM_DMA_ID_x = getTimDmaId(timerChannel);

  gpioHtim->hdma[TIM_DMA_ID_x]->XferHalfCpltCallback = halfCplt;
  gpioHtim->hdma[TIM_DMA_ID_x]->XferCpltCallback = cplt;

  /* Set the DMA error callback */
  gpioHtim->hdma[TIM_DMA_ID_x]->XferErrorCallback = gpioErrorCb;

  /* Enable the DMA channel */
  if (HAL_DMA_Start_IT(gpioHtim->hdma[TIM_DMA_ID_x], (uint32_t)source, (uint32_t)dest, kDoubleBufferSize) != HAL_OK)
  {
    return -1;
  }

  /* Enable the TIM Capture/Compare DMA request */
  uint32_t TIM_DMA_x = getTimDmaRequestEnableBit(timerChannel);
  __HAL_TIM_ENABLE_DMA(gpioHtim, TIM_DMA_x);
  return 0;
}

static int gpioHighRateInit()
{
  if(gpioHighRateInitDma(kGpioHighRateOutTimerChannel, gGpioOut, &gGpioHighRateOutBank->BSRR, gpioHighRateOutHalfCpltCb, gpioHighRateOutCpltCb))
    return -1;
  if(gpioHighRateInitDma(kGpioHighRateInTimerChannel, &gGpioHighRateInBank->IDR, (uint32_t*)gGpioIn, gpioHighRateInHalfCpltCb, gpioHighRateInCpltCb))
    return -1;
  return 0;
}

static int gpioHighRateStart()
{
  // finally, enable the timer without DMA (DMA initialised above)
  // TODO: there is a bit of overhead in HAL_TIM_PWM_Start and they are actually both enabling the clock in EN1
  // Could be refactored to make it start earlier, or even better look at other ways of starting it.
#ifdef GPIO_OUT_USE_DMA
  if(HAL_TIM_PWM_Start(gpioHtim, getTimChannel(kGpioHighRateOutTimerChannel)))
    return -1;
#endif // GPIO_OUT_USE_DMA
#ifdef GPIO_IN_USE_DMA
  if(HAL_TIM_PWM_Start(gpioHtim, getTimChannel(kGpioHighRateInTimerChannel)))
    return -1;
#endif // GPIO_IN_USE_DMA
  return 0;
}
#endif // defined(GPIO_OUT_USE_DMA) || defined (GPIO_IN_USE_DMA)

static void render(uint8_t end);

#if defined(DAC_USE_DMA) || defined(ADC_USE_DMA) || defined(GPIO_OUT_USE_DMA) || defined(GPIO_IN_USE_DMA)
static void streamComplete(Stream stream, uint8_t end)
{
  static uint8_t streamStates[kNumStreams];
  streamStates[stream] = end;
  uint8_t ready = 1;
  for(unsigned int n = 1; n < kNumStreams; ++n)
  {
    if(streamStates[n] != streamStates[0])
    {
      ready = 0;
      break;
    }
  }
  if(ready)
    render(end);
}

static void render(uint8_t end)
{
#ifdef TOGGLE_DEBUG_PINS
  HAL_GPIO_WritePin(DEBUG3_GPIO_Port, DEBUG3_Pin, GPIO_PIN_SET);
#endif // TOGGLE_DEBUG_PINS
#ifdef DAC_USE_DMA
#ifdef TRILL_RACK_INTERFACE
  tr_process(NULL);
  // set DACx based on gDacNext[x]
  for(unsigned int channel = 0; channel < kDacNumChannels; ++channel)
  {
    static float past[kDacNumChannels];
//    float tmp = gDacOutputs[channel][kDoubleBufferSize / 2 + kDoubleBufferSize / 2 * (!end) - 1];
    float tmp = past[channel];
    float alpha = 0.9;
    for(unsigned int n = 0; n < kDoubleBufferSize / 2; ++n)
    {
      tmp = tmp * alpha + gDacNext[channel] * (1.f - alpha);
      gDacOutputs[channel][n + end * kDoubleBufferSize / 2] = tmp;
    }
    past[channel] = tmp;
  }
#else // TRILL_RACK_INTERFACE
  // set DAC0 based on gDacNext[0]
  unsigned int channel = 0;
//  for(unsigned int channel = 0; channel < kDacNumChannels; ++channel)
  {
    float tmp = gDacOutputs[channel][kDoubleBufferSize / 2 + kDoubleBufferSize / 2 * (!end) - 1];
    float alpha = 0.9;
    for(unsigned int n = 0; n < kDoubleBufferSize / 2; ++n)
    {
      tmp = tmp * alpha + gDacNext[channel] * (1.f - alpha);
      gDacOutputs[channel][n + end * kDoubleBufferSize / 2] = tmp;
    }
  }
#if 0
  {
    // set DAC1 to echo ADC
    channel = 1;
    for(unsigned int n = 0; n < kDoubleBufferSize / 2; ++n)
    {
      unsigned int idx = n + end * kDoubleBufferSize / 2;
  #ifdef ADC_USE_DMA
      gDacOutputs[channel][idx] = gAdcInputs[idx];
  #endif // ADC_USE_DMA
    }
  }
#else
  {
    //set DAC1 to otuput a sinewave
    channel = 1;
    static float amp = 0;
    static float ampSig = 1;
    static float phase = 0;
    const float kAmpMax = 0.1;
    const float kAmpMin = 0;
    for(unsigned int n = 0; n < kDoubleBufferSize / 2; ++n)
    {
      unsigned int idx = n + end * kDoubleBufferSize / 2;
      phase += 2.f * (float)M_PI * 400.f / 40000.f;
      if(phase > M_PI)
        phase -= 2.f * (float)M_PI;
      float out = sinf(phase) * amp;
      amp += ampSig * 0.1f / 40000.f;
      if(amp > kAmpMax) {
        ampSig = -1;
        amp = kAmpMax;
      }
      if(amp < kAmpMin) {
        ampSig = 1;
        amp = 0;
      }
      gDacOutputs[channel][idx] = (out * 0.5f + 0.5f) * (1 << 12) + 0.5f;
    }
  }
#endif
#endif // TRILL_RACK_INTERFACE
#endif // DAC_USE_DMA
#ifdef GPIO_OUT_USE_DMA
  { // output a clock on kTestChannel
    enum { kOnes = 5, kZeros = 1};
    enum { kTestChannel = 6 };
    for(unsigned int n = 0; n < kOnes; ++n)
      digitalWrite(end, n, kTestChannel, 1);
    for(unsigned int n = kOnes; n < kOnes + kZeros; ++n)
      digitalWrite(end, n, kTestChannel, 0);
    const int kNumBits = kDoubleBufferSize / 2 - kOnes - kZeros;
    static int count = 0;
    for(int n = 0; n < kNumBits; ++n)
    {
      unsigned int frame = n + kOnes + kZeros;
      digitalWrite(end, frame, kTestChannel, (count >> n) & 1);
    }
    ++count;
    if(count >= (1 << kNumBits))
      count = 0;
  }
#endif // GPIO_USE_DMA
#ifdef TOGGLE_DEBUG_PINS
  HAL_GPIO_WritePin(DEBUG3_GPIO_Port, DEBUG3_Pin, GPIO_PIN_RESET);
#endif // TOGGLE_DEBUG_PINS
}
#endif // any DMA

#ifdef NEOPIXEL_USE_TIM

#define NEOPIXEL_USE_CLASS
#include <stdlib.h> // atoi below

#ifdef NEOPIXEL_USE_CLASS
extern "C" {
  int npSetup(void);
  ssize_t npSend(const uint8_t* rgb, size_t length);
  void npSendColorToAll(uint32_t val);
  void npDone();
  uint8_t npReady();
};

#else // NEOPIXEL_USE_CLASS
volatile uint8_t gNpBusyFlag = 0;

void WS2812_Send (uint32_t color)
{

  enum { kNumPixels = 16};
  enum { kNumBitsPerPixel =  24 };
  enum { kNumBits = kNumPixels * kNumBitsPerPixel };
  enum { kNumData = kNumBits + 2 }; // one leading and one trailing 0-valued
  enum { kDataStart = 1 }; // leading 0 byte
  static uint32_t pwmData[kNumData]; // this should be uint16_t if using a timer other than TIM2 and the DMA settings should be changed to half-word in that case, too

  // see https://controllerstech.com/pwm-with-dma-in-stm32/, but adapted for TIM2 (32 bit CCR).
  if(gNpBusyFlag)
    return;
  gNpBusyFlag = 1;
  for(unsigned int n = 0; n < kDataStart; ++n)
    pwmData[n] = 0;
  for(unsigned int n = 0; n < kNumPixels; ++n)
  {
    for (int i = kNumBitsPerPixel; i >= 0; --i)
    {
      unsigned int idx = kNumBitsPerPixel * n + i + kDataStart;
      if (color & (1 << ((7 - (i % 8)) + ((i / 8) * 8) ))) // don't ask
        pwmData[idx] = 66;
      else
        pwmData[idx] = 33;
    }
  }
  pwmData[kNumData - 1] = 0; // ensure the last PWM value is 0, so if the DMA callback arrives late, we have stopped sending out stuff already.
  HAL_Delay(1); // min delay between repetitions
  neoPixelHtim.Instance->CCR2 = 0;
  HAL_TIM_PWM_Start_DMA(&neoPixelHtim, TIM_CHANNEL_2, (uint32_t *)pwmData, kNumData);
}
#endif // NEOPIXEL_USE_CLASS

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
#ifdef TRILL_RACK_INTERFACE
  tr_snpDone();
#else // TRILL_RACK_INTERFACE
#ifdef NEOPIXEL_USE_CLASS
  npDone();
#else // NEOPIXEL_USE_CLASS
  gNpBusyFlag = 0;
#endif // NEOPIXEL_USE_CLASS
#endif // TRILL_RACK_INTERFACE
  fprintf(stderr, "TIM error %p\n", htim);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if(&neoPixelHtim == htim)
  {
#ifdef TRILL_RACK_INTERFACE
  tr_snpDone();
#else // TRILL_RACK_INTERFACE
#ifdef NEOPIXEL_USE_CLASS
    npDone();
#else // NEOPIXEL_USE_CLASS
    HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);
    gNpBusyFlag = 0;
#endif // NEOPIXEL_USE_CLASS
#endif // TRILL_RACK_INTERFACE
  }
}
#endif // NEOPIXEL_USE_TIM
int TrillRackApplication()
{
  RetargetInit(&dbgHuart);
  printf("Booted\n\r");
  // Calibrate The ADC On Power-Up For Better Accuracy
  HAL_ADCEx_Calibration_Start(&adcHandle, ADC_SINGLE_ENDED);
  // turn on the SDA pullup
  HAL_GPIO_WritePin(PSOC_PULLUP_SDA_GPIO_Port, PSOC_PULLUP_SDA_Pin, GPIO_PIN_SET);
  // powercycle the Trill (via external inverting mosfet)
  HAL_GPIO_WritePin(TRILL_3V3_GPIO_Port, TRILL_3V3_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(TRILL_3V3_GPIO_Port, TRILL_3V3_Pin, GPIO_PIN_RESET);
  HAL_Delay(250); // wait for device to be receptive to I2C transactions

#ifdef TRILL_RACK_INTERFACE
  int ret = tr_setup();
  printf("tr_setup: %x\n\r", ret);
  if(ret <= 0)
    Error_Handler();
  gI2cAddress = ret << 1;
#else // TRILL_RACK_INTERFACE
#ifdef NEOPIXEL_USE_CLASS
  npSetup();
#endif // NEOPIXEL_USE_CLASS
#ifdef TRILL_USE_CLASS
  int ret;
  if((ret = trillSetup()))
  {
    fprintf(stderr, "Error setting up Trill: %d\n", ret);
    return 1;
  }
#else // TRILL_USE_CLASS
  uint8_t identifyBuf[] = {kOffsetCommand, kCommandIdentify};
  int ret = HAL_I2C_Master_Transmit(&trillHi2c, gI2cAddress, identifyBuf, sizeof(identifyBuf), kTimeout);
  if(HAL_OK != ret) {
    fprintf(stderr, "Error: send identify command\n\r");
    return 1;
  }
  HAL_Delay(10);
  uint8_t receiveBuffer[4];
  ret = HAL_I2C_Master_Receive(&trillHi2c, gI2cAddress, receiveBuffer, sizeof(receiveBuffer), kTimeout);
  if(HAL_OK != ret) {
    fprintf(stderr, "Error: receive identify command\n\r");
    return 1;
  }
  printf("identify: %#4x %#4x %#4x %#4x\n\r", receiveBuffer[0], receiveBuffer[1], receiveBuffer[2], receiveBuffer[3]);
  HAL_Delay(10);
  uint8_t diffBuf[] = {kOffsetCommand, kCommandMode, kModeCentroid};
  ret = HAL_I2C_Master_Transmit(&trillHi2c, gI2cAddress, diffBuf, sizeof(diffBuf), kTimeout);
  if(HAL_OK != ret) {
    fprintf(stderr, "Error: send mode command\n\r");
    return 1;
  }
  // update baseline
  uint8_t updateBaselineBuffer[] = {kOffsetCommand, kCommandBaselineUpdate};
  ret = HAL_I2C_Master_Transmit(&trillHi2c, gI2cAddress, updateBaselineBuffer, sizeof(updateBaselineBuffer), kTimeout);
  if(HAL_OK != ret) {
    fprintf(stderr, "Error: send update baseline command\n\r");
    return 1;
  }
  // prepare to read data
  uint8_t transmitBuffer[] = {kOffsetData};
  ret = HAL_I2C_Master_Transmit(&trillHi2c, gI2cAddress, transmitBuffer, sizeof(transmitBuffer), kTimeout);
  if(HAL_OK != ret) {
    fprintf(stderr, "Error: prepare to read command\n\r");
    return 1;
  }
#endif// TRILL_USE_CLASS
#endif // TRILL_RACK_INTERFACE
#ifdef I2C_USE_DMA
  ret = HAL_I2C_Master_Receive_DMA(&trillHi2c, gI2cAddress, gI2cDmaRecv, gI2cDmaRecvSize);
  if(HAL_OK != ret)
  {
    fprintf(stderr, "I2C_Master_Receive_DMA failed: %d\n", ret);
    return 1;
  }
#endif // I2C_USE_DMA
#ifdef DAC_USE_DMA
  ret = HAL_DAC_Start_DMA(&dac0Handle, dac0Channel, (uint32_t*)gDacOutputs[0], kDoubleBufferSize, DAC_ALIGN_12B_R);
  if(HAL_OK != ret)
  {
    fprintf(stderr, "DAC_Start_DMA 0 failed: %d\n", ret);
    return 1;
  }
  ret = HAL_DAC_Start_DMA(&dac1Handle, dac1Channel, (uint32_t*)gDacOutputs[1], kDoubleBufferSize, DAC_ALIGN_12B_R);
  if(HAL_OK != ret)
  {
    fprintf(stderr, "DAC_Start_DMA 1 failed: %d\n", ret);
    return 1;
  }
#else // DAC_USE_DMA
  ret = HAL_DAC_Start(&dac0Handle, dac0Channel);
  if(HAL_OK != ret)
  {
    fprintf(stderr, "Error: HAL_DAC_Start()\n\r");
    return 1;
  }
  ret = HAL_DAC_Start(&dac1Handle, dac1Channel);
  if(HAL_OK != ret)
  {
    fprintf(stderr, "Error: HAL_DAC_Start()\n\r");
    return 1;
  }
#endif // DAC_USE_DMA
#ifdef ADC_USE_DMA
  ret = HAL_ADC_Start_DMA(&adcHandle, (uint32_t*)gAdcInputs, kDoubleBufferSize);
  if(HAL_OK != ret)
  {
    fprintf(stderr, "ADC_Start_DMA failed: %d\n", ret);
    return 1;
  }
#else // ADC_USE_DMA
  HAL_ADC_Start(&adcHandle);
#endif// ADC_USE_DMA
#if defined(GPIO_OUT_USE_DMA) || defined(GPIO_IN_USE_DMA)
  if(gpioHighRateInit())
  {
    fprintf(stderr, "gpioHighRateInit failed\n");
    return 1;
  }
#endif // defined(GPIO_OUT_USE_DMA) || defined(GPIO_IN_USE_DMA)

#if defined(GPIO_OUT_USE_DMA) || defined(GPIO_IN_USE_DMA)
  if(gpioHighRateStart())
  {
    fprintf(stderr, "gpioHighRateStart failed\n");
    return 1;
  }
#endif // defined(GPIO_OUT_USE_DMA) || defined(GPIO_IN_USE_DMA)
#if defined(DAC_USE_DMA) || defined (ADC_USE_DMA)
  ret = HAL_TIM_Base_Start(&dacAdcHtim);
  if(HAL_OK != ret)
  {
    fprintf(stderr, "TIM_Base_Start for DAC/ADC failed: %d\n", ret);
    return 1;
  }
#endif // DAC_USE_DMA || ADC_USE_DMA
  while (1)
  {
#ifdef TRILL_RACK_INTERFACE
    // not much to do here ...
#else // TRILL_RACK_INTERFACE
#ifdef NEOPIXEL_USE_TIM
#ifdef NEOPIXEL_USE_CLASS
    if(npReady())
#else // NEOPIXEL_USE_CLASS
    if(!gNpBusyFlag)
#endif // NEOPIXEL_USE_CLASS
     {
       // updated neopixels
 #if 1 // interactive LEDs
       static int val = 0;
       char data[2] = {0};
       HAL_StatusTypeDef status = HAL_UART_Receive(&dbgHuart, (uint8_t*)data, 1, 0);
       if(HAL_OK == status)
       {
         printf("%s", data);
         char mybuf[10];
         static int i = 0;
         if(data[0] == '\n' || data[0] == '\r' || i >= sizeof(mybuf) - 1)
         {
           mybuf[i] = 0;
           val = atoi(mybuf);
           printf("Sending %d\n\r", val);
           i = 0;
#ifdef NEOPIXEL_USE_CLASS
           npSendColorToAll(val);
#else // NEOPIXEL_USE_CLASS
           WS2812_Send(val);
#endif // NEOPIXEL_USE_CLASS
         } else {
           mybuf[i] = data[0];
           ++i;
         }
       }
 #else
       static uint8_t count = 1;
       static uint8_t shift = 0;
       static uint8_t inc = 1;
#ifdef NEOPIXEL_USE_CLASS
       uint32_t val = count << shift;
       npSendColorToAll(val);
#else // NEOPIXEL_USE_CLASS
       WS2812_Send((count << shift));
#endif// NEOPIXEL_USE_CLASS
       if(count == 255)
         inc = 0;
       if(count == 0)
       {
         inc = 1;
         shift += 8;
       }
       if(shift >= 24)
         shift = 0;
       if(inc)
         ++count;
       else
         --count;
 #endif
     }
#endif // NEOPIXEL_USE_TIM
#ifndef TRILL_USE_CLASS
    uint8_t receiveBuffer[kNumTouches * 2 * 2];
#endif // TRILL_USE_CLASS
#ifdef I2C_USE_DMA
    // TODO: does the below make sense?
    memcpy(receiveBuffer, gI2cLatestRecv, sizeof(receiveBuffer));
#else // I2C_USE_DMA
#ifdef TRILL_USE_CLASS
    ret = trillRead();
#else // TRILL_USE_CLASS
    ret = HAL_I2C_Master_Receive(&trillHi2c, gI2cAddress, receiveBuffer, sizeof(receiveBuffer), kTimeout);
#endif // TRILL_USE_CLASS
    if(HAL_OK != ret) {
      fprintf(stderr, "Error: blocking receive\n\r");
      return 1;
    }
#endif // I2C_USE_DMA
    uint16_t firstLocation = 0;
    uint16_t firstSize = 0;
//#define RUN_PRINT
#ifdef RUN_PRINT
    uint8_t written = 0;
#endif // RUN_PRINT
#ifdef TRILL_USE_CLASS
    if(trillNumTouches())
    {
      firstLocation = 4096 * trillTouchLocation(0);
      firstSize = 4096 * trillTouchSize(0);
    }
#else // TRILL_USE_CLASS
    for(unsigned int n = 0; n < kNumTouches; ++n)
    {
      uint16_t location = ((receiveBuffer[2 * n] << 8) + receiveBuffer[2 * n + 1]);
      uint16_t size = ((receiveBuffer[2 * n + kNumTouches * 2] << 8) + receiveBuffer[2 * n + 1 + kNumTouches * 2]);
      if(location != 0xffff)
      {
        if(0 == n)
        {
          firstLocation = location;
          firstSize = size;
        }
#ifdef RUN_PRINT
        written = 1;
        printf("[%d] %d %d, ", n, location, size);
#endif // RUN_PRINT
      }
    }
#endif // TRILL_USE_CLASS
#ifdef DAC_USE_DMA
    gDacNext[0] = firstLocation;
    gDacNext[1] = firstSize > 4095 ? 4095 : firstSize;
#else // DAC_USE_DMA
    HAL_DAC_SetValue(&dac0Handle, dac0Channel, DAC_ALIGN_12B_R, firstLocation);
    HAL_DAC_SetValue(&dac1Handle, dac1Channel, DAC_ALIGN_12B_R, firstSize);
#endif //DAC_USE_DMA
#ifndef ADC_USE_DMA
    ret = HAL_ADC_PollForConversion(&adcHandle, 10);
    if(HAL_OK != ret)
    {
      fprintf(stderr, "Error: ADC poll for conversion\n\r");
      return 1;
    }
    uint32_t adc = HAL_ADC_GetValue(&adcHandle);
    HAL_ADC_Start(&adcHandle);
#endif // ADC_USE_DMA

#ifdef RUN_PRINT
    if(written || 1)
    {
      printf("%ld \n\r", adc);
    }

    for(volatile int n = 0; n < 100000; ++n)
      ;
#endif // RUN_PRINT
#endif // TRILL_RACK_INTERFACE

  }
}
