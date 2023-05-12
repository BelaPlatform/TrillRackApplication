#include <TrillRackApplication_bsp.h>
#include <stdio.h>

void test_dac_nodma()
{
  printf("test_dac_nodma\n\r");
  // requires disabling triggering for DAC
  HAL_DAC_Start(&dac0Handle, dac0Channel);
  HAL_DAC_Start(&dac1Handle, dac1Channel);
  uint16_t i = 1; // 1 instead of 0 so we don't get caught by the if below the first time
  int sign = 1;
  while (1)
  {
    int ret = HAL_DAC_SetValue(&dac0Handle, dac0Channel, DAC_ALIGN_12B_R, i);
    if(HAL_OK != ret)
      printf("Error during DAC_SetValue 0: %d\n\r", ret);
    ret = HAL_DAC_SetValue(&dac1Handle, dac1Channel, DAC_ALIGN_12B_R, i);
    if(HAL_OK != ret)
      printf("Error during DAC_SetValue 1: %d\n\r", ret);
    if(i >= 4095 || i == 0)
    {
      sign = -sign;
    }
    i += sign;
  }
}

enum { kDoubleBufferSize = 4096 };
static uint16_t gDacOutputs[2][kDoubleBufferSize];


void test_dac_dma()
{
  printf("test_dac_dma\n\r");
  for (unsigned int n = 0; n < kDoubleBufferSize; ++n)
  {
    gDacOutputs[0][n] = n % 4096;
    gDacOutputs[1][n] = (n + 1024) % 4096;
  }
  int ret = HAL_DAC_Start_DMA(&dac0Handle, dac0Channel, (uint32_t*)gDacOutputs[0], kDoubleBufferSize, DAC_ALIGN_12B_R);
  if(HAL_OK != ret)
    fprintf(stderr, "Error during DAC_Start_DMA 0: %d\n\r", ret);
  HAL_TIM_Base_Start(&dacAdcHtim);
  if(HAL_OK != ret)
      fprintf(stderr, "Error during TIM_Base_Start %d\n\r", ret);

//  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)Wave_LUT, 128, DAC_ALIGN_12B_R);
//  HAL_TIM_Base_Start_IT(&htim6);
}
