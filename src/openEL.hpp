#ifndef OPENEL_HPP
#define OPENEL_HPP

#include "HAL4RT.hpp"
#include "HALId.hpp"
#include "ReturnCode.hpp"
#include "HALComponent.hpp"
#include "HALObserver.hpp"
#include "EventTimer.hpp"
#include "TimerObserver.hpp"
#include "Property.hpp"

typedef union halArgument_ut
{
    int64_t numI64;
    int32_t num;
    struct { HALFLOAT_T value; int32_t _dummy; int32_t num; } FI;
} HAL_ARGUMENT_T;

typedef union halArgumentDevice_ut HAL_ARGUMENT_DEVICE_T;

typedef struct HalFncTbl_st HAL_FNCTBL_T;

class HALComponent;
class HALObserver;

typedef struct HalHandler_st
{
    uint32_t swUsed;
    HALComponent *pHalComponent;
    HAL_FNCTBL_T *pFncTbl;
} HAL_HANDLER_T;

#define HAL_SZ_HANDLER_TBL  (256)
#define HAL_MSK_HANDLER_TBL (HAL_SZ_HANDLER_TBL - 1)

extern HAL_HANDLER_T HalHandlerTbl[HAL_SZ_HANDLER_TBL];

typedef struct HalReg_st
{
    int32_t deviceKindID;
    int32_t vendorID;
    int32_t productID;
    HAL_FNCTBL_T *pFncTbl;
    int32_t szHalComponent;
} HAL_REG_T;

extern const HAL_REG_T HalRegTbl[];
extern const int32_t hal_szRegTbl;

typedef struct HalFncTbl_st
{
    /* 0x00 */ ReturnCode (*pFncInit)(HALComponent*); /**< Initialize */   \
    /* 0x01 */ ReturnCode (*pFncReInit)(HALComponent*); /**< ReInit */	  \
    /* 0x02 */ ReturnCode (*pFncFinalize)(HALComponent*); /**< Finalize */		\
    /* 0x03 */ ReturnCode (*pFncAddObserver)(HALComponent*, HALObserver **halObserver); /**< AddObserver */ \
    /* 0x04 */ ReturnCode (*pFncRemoveObserver)(HALComponent*, HALObserver **halObserver); /**< RemoveObserver */ \
    /* 0x05 */ ReturnCode (*pFncGetProperty)(HALComponent*, Property **property); /**< GetProperty */ \
    /* 0x06 */ ReturnCode (*pFncGetTime)(HALComponent*, unsigned int **); /**< GetTime */ \
    /* 0x07 */ ReturnCode (*pFncDummy07)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */

    /* 0x08 */ ReturnCode (*pFncDummy08)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x09 */ ReturnCode (*pFncDummy09)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x0A */ ReturnCode (*pFncDummy0A)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x0B */ ReturnCode (*pFncDummy0B)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x0C */ ReturnCode (*pFncDummy0C)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x0D */ ReturnCode (*pFncDummy0D)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x0E */ ReturnCode (*pFncDummy0E)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x0F */ ReturnCode (*pFncDummy0F)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */

    /* 0x10 */ ReturnCode (*pFncDummy10)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x11 */ ReturnCode (*pFncDummy11)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x12 */ ReturnCode (*pFncDummy12)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x13 */ ReturnCode (*pFncDummy13)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x14 */ ReturnCode (*pFncDummy14)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x15 */ ReturnCode (*pFncDummy15)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x16 */ ReturnCode (*pFncDummy16)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */
    /* 0x17 */ ReturnCode (*pFncDummy17)(HALComponent*, HAL_ARGUMENT_T*); /**< Reserved */

    /* 0x18 */ ReturnCode (*pFncSetValue)(HALComponent *, int, float);    /**< SetValue */
    /* 0x19 */ ReturnCode (*pFncGetValue)(HALComponent *, int, float **); /**< GetValue */
    /* 0x1A */ ReturnCode (*pFncGetValueList)(HALComponent*, float **, int **); /**< GetValueList */
    /* 0x1B */ ReturnCode (*pFncGetTimedValueList)(HALComponent*, float **, unsigned int **, int **); /**< GetTimedValueList */
    /* 0x1C */ ReturnCode (*pFncDeviceVendor1C)(HALComponent*, HAL_ARGUMENT_T*,HAL_ARGUMENT_DEVICE_T *); /**< Device Vendor Function */
    /* 0x1D */ ReturnCode (*pFncDeviceVendor1D)(HALComponent*, HAL_ARGUMENT_T*,HAL_ARGUMENT_DEVICE_T *); /**< Device Vendor Function */
    /* 0x1E */ ReturnCode (*pFncDeviceVendor1E)(HALComponent*, HAL_ARGUMENT_T*,HAL_ARGUMENT_DEVICE_T *); /**< Device Vendor Function */
    /* 0x1F */ ReturnCode (*pFncDeviceVendor1F)(HALComponent*, HAL_ARGUMENT_T*,HAL_ARGUMENT_DEVICE_T *); /**< Device Vendor Function */
} HAL_FNCTBL_T;

HALComponent* HalCreate(int32_t vendorID, int32_t productID, int32_t instanceID);
void HalDestroy(HALComponent *halComponent);

ReturnCode HalInit(HALComponent *halComponent);
ReturnCode HalReInit(HALComponent *halComponent);
ReturnCode HalFinalize(HALComponent *halComponent);
ReturnCode HalAddObserver(HALComponent *halComponent, HALObserver *halObserver);
ReturnCode HalRemoveObserver(HALComponent *halComponent, HALObserver *halObserver);
ReturnCode HalGetProperty(HALComponent *halComponent, Property *property);
ReturnCode HalGetTime(HALComponent *halComponent, int32_t *timeValue);

#endif // OPENEL_HPP
