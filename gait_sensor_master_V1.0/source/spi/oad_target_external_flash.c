/*******************************************************************************

 @file  oad_target_external_flash.c

 @brief This file contains the external flash target implementation of the
        OAD profile.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 *******************************************************************************
 
 Copyright (c) 2014-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "board.h"
#include "ExtFlash.h"

/*******************************************************************************
 * PRIVATE VARIABLES
 */
static bool isOpen = false;

/*******************************************************************************
 * PRIVATE FUNCTIONS
 */
#define HAL_FLASH_PAGE_SIZE       4096


/*******************************************************************************
 * FUNCTIONS
 */

/*******************************************************************************
 * @fn      OADTarget_open
 *
 * @brief   Open an OAD target for download.
 *
 * @param   none
 *
 * @return  TRUE if OAD target successfully opened
 */
uint8_t OADTarget_open(void)
{
    if (!isOpen)
    {
        isOpen = ExtFlash_open();
    }

    return isOpen ? TRUE : FALSE;
}


/*******************************************************************************
 * @fn      OADTarget_close
 *
 * @brief   Close an OAD target after a download has finished
 *
 * @param   none
 *
 * @return  none
 */
void OADTarget_close(void)
{
  if (isOpen)
  {
    isOpen = false;
    ExtFlash_close();
  }
}

/*******************************************************************************
 * @fn      OADTarget_hasExternalFlash
 *
 * @brief   Check if the target has external flash
 *
 * @param   none
 *
 * @return  always TRUE
 */
uint8_t OADTarget_hasExternalFlash(void)
{
  return true;
}


/*******************************************************************************
 * @fn      OADTarget_readFlash
 *
 * @brief   Read data from flash.
 *
 * @param   page   - page to read from in flash
 * @param   offset - offset into flash page to begin reading
 * @param   pBuf   - pointer to buffer into which data is read.
 * @param   len    - length of data to read in bytes.
 *
 * @return  None.
 */
void OADTarget_readFlash(uint32_t page, uint8_t *pBuf,
                         uint16_t len)
{
  ExtFlash_read(page, len, pBuf);
}

/*******************************************************************************
 * @fn      OADTarget_writeFlash
 *
 * @brief   Write data to flash.
 *
 * @param   page   - page to write to in flash
 * @param   offset - offset into flash page to begin writing
 * @param   pBuf   - pointer to buffer of data to write
 * @param   len    - length of data to write in bytes
 *
 * @return  None.
 */
void OADTarget_writeFlash(uint32_t page, uint8_t *pBuf,
                          uint16_t len)
{
  ExtFlash_write(page, len, pBuf);
}

/*********************************************************************
 * @fn      OADTarget_eraseFlash
 *
 * @brief   Erase selected flash page.
 *
 * @param   page - the page to erase.
 *
 * @return  None.
 */
void OADTarget_eraseFlash(uint32_t page)
{
  ExtFlash_erase(page, HAL_FLASH_PAGE_SIZE);
}

/*********************************************************************
 * @fn      OAD_imgIdentifyWrite
 *
 * @brief   Process the Image Identify Write.  Determined if the image
 *          header identified here should or should not be downloaded by
 *          this application.
 *
 * @param   connHandle - connection message was received on
 * @param   pValue     - pointer to image header data
 *
 * @return  none
 */
void HwFlashErase(uint32_t page)
{
  // Open the target interface
  if (OADTarget_open())
  {

      OADTarget_eraseFlash(page);
  }
}

void HwFlashErase_All(void)
{
  //Open the target interface
  if(OADTarget_open())
  {
    ExtFlash_eraseall();
  }
}
/*********************************************************************
 * @fn      OAD_imgBlockWrite
 *
 * @brief   Process the Image Block Write.
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to data to be written
 *
 * @return  none
 */
void HwFlashWrite(uint32_t page, uint8_t *pBuf, uint16_t len)
{
  // Open the target interface
  if (OADTarget_open())
  {
     OADTarget_writeFlash(page, pBuf, len);
  }
}

void HwFlashRead(uint32_t page, uint8_t *pBuf, uint16_t len)
{
  // Open the target interface
  if (OADTarget_open())
  {
     OADTarget_readFlash(page, pBuf, len);
  }
}
