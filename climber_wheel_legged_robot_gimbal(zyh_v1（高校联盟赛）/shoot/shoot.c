/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "shoot.h"

#if SHOOT_TYPE != SHOOT_NONE

#include <stdlib.h>

// ShootApi_t shoot = {
//     .GetStatus = GetStatus,
//     .GetDuration = GetDuration,
// };

ShootApi_t shoot = {
    .GetStatus = NULL,
    .GetDuration = NULL,
};
#endif  // SHOOT_TYPE
/*------------------------------ End of File ------------------------------*/