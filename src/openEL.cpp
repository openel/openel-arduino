#include "openEL.hpp"

HAL_HANDLER_T HalHandlerTbl[HAL_SZ_HANDLER_TBL];
static int32_t HalIdxHandlerTbl;

HALComponent* HalCreate(int32_t deviceKindID, int32_t vendorID, int32_t productID, int32_t instanceID)
{
    int32_t i;
    int32_t idx = -1;
    HAL_HANDLER_T *pHandler;
    HALComponent *pHalComponent;
    const HAL_REG_T *pReg;

    for (i = 0; i < HAL_SZ_HANDLER_TBL; i++)
    {
	HalIdxHandlerTbl = (HalIdxHandlerTbl + 1) & HAL_MSK_HANDLER_TBL;
	if (0 == HalHandlerTbl[HalIdxHandlerTbl].swUsed )
	{
	    idx = HalIdxHandlerTbl;
	    break;
	}
    }
    if (-1 == idx) return 0;
    pHandler = &HalHandlerTbl[HalIdxHandlerTbl];

    idx = -1;
    for (i = 0; i < hal_szRegTbl; i++)
    {
	if ((deviceKindID == HalRegTbl[i].deviceKindID && vendorID == HalRegTbl[i].vendorID) && (productID == HalRegTbl[i].productID))
	{
	    idx = i;
	    break;
	}
    }
    if (-1 == idx) return 0;
    pReg = &HalRegTbl[idx];

    pHalComponent = new HALComponent;
    pHalComponent->handle = HalIdxHandlerTbl;
    pHalComponent->hALId.deviceKindId = deviceKindID;
    pHalComponent->hALId.vendorId = vendorID;
    pHalComponent->hALId.productId = productID;
    pHalComponent->hALId.instanceId = instanceID;

    pHandler->swUsed = 1;
    pHandler->pHalComponent = pHalComponent;
    pHandler->pFncTbl = pReg->pFncTbl;

    return pHalComponent;
}

void HalDestroy(HALComponent *halComponent)
{
    HAL_HANDLER_T *pHandler;

    pHandler = &HalHandlerTbl[halComponent->handle];
    pHandler->swUsed = 0;
    delete halComponent;
}
