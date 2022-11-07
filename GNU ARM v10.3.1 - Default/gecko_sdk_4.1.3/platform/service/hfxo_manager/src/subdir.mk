################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2/platform/service/hfxo_manager/src/sl_hfxo_manager.c \
C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2/platform/service/hfxo_manager/src/sl_hfxo_manager_hal_s2.c 

OBJS += \
./gecko_sdk_4.1.3/platform/service/hfxo_manager/src/sl_hfxo_manager.o \
./gecko_sdk_4.1.3/platform/service/hfxo_manager/src/sl_hfxo_manager_hal_s2.o 

C_DEPS += \
./gecko_sdk_4.1.3/platform/service/hfxo_manager/src/sl_hfxo_manager.d \
./gecko_sdk_4.1.3/platform/service/hfxo_manager/src/sl_hfxo_manager_hal_s2.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.1.3/platform/service/hfxo_manager/src/sl_hfxo_manager.o: C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2/platform/service/hfxo_manager/src/sl_hfxo_manager.c gecko_sdk_4.1.3/platform/service/hfxo_manager/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DEFR32MG21A020F1024IM32=1' '-DSL_APP_PROPERTIES=1' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DCORTEXM3=1' '-DCORTEXM3_EFM32_MICRO=1' '-DCORTEXM3_EFR32=1' '-DPHY_RAIL=1' '-DPLATFORM_HEADER="platform-header.h"' '-DSL_LEGACY_HAL_ENABLE_WATCHDOG=1' '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' '-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' '-DCUSTOM_TOKEN_HEADER="sl_token_manager_af_token_header.h"' '-DUSE_NVM3=1' '-DUC_BUILD=1' '-DSL_ZIGBEE_LEAF_STACK=1' '-DEMBER_MULTI_NETWORK_STRIPPED=1' '-DSL_ZIGBEE_PHY_SELECT_STACK_SUPPORT=1' '-DCONFIGURATION_HEADER="app/framework/util/config.h"' -I"C:\Users\HaraldLIngebrigtsen\SimplicityStudio\Strombergs\strombergs_node_test\config" -I"C:\Users\HaraldLIngebrigtsen\SimplicityStudio\Strombergs\strombergs_node_test\autogen" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/Device/SiliconLabs/EFR32MG21/Include" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/common/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/bootloader" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/bootloader/api" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/cli/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/cli/src" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/CMSIS/Core/Include" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/driver/debug/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/device_init/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/emdrv/common/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/emlib/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/emdrv/gpiointerrupt/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/hfxo_manager/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/iostream/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/legacy_hal/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/legacy_hal_wdog/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/legacy_printf/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/sl_mbedtls_support/config" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/sl_mbedtls_support/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/mbedtls/include" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/mbedtls/library" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/emdrv/nvm3/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/power_manager/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/printf" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/printf/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/sl_psa_driver/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/sl_psa_driver/inc/public" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/common" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/protocol/ble" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/protocol/ieee802154" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/protocol/zwave" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/chip/efr32/efr32xg2x" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin/rail_util_ieee802154" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin/pa-conversions" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin/pa-conversions/efr32xg21" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin/rail_util_pti" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin/rail_util_rf_path" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/se_manager/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/se_manager/src" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/silicon_labs/silabs_core/memory_manager" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/common/toolchain/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/system/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/sleeptimer/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/token_manager/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/platform/micro" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/include" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/common" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/em260" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/util/serial" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/service-function" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/counters" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/framework" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/debug-print" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/network-steering" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/scan-dispatch" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/signature-decode" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/gp" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/zigbee" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/mac/rail_mux" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/mac" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/silicon_labs/silabs_core" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/core" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/mac" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/update-tc-link-key" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/include" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/util" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/security" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/util/zigbee-framework" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/util/counters" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/cli" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/util/common" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/util/security" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/ota-storage-common" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse -fno-builtin-printf -fno-builtin-sprintf -Wno-unused-parameter -Wno-missing-field-initializers -Wno-missing-braces -Wno-format -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.1.3/platform/service/hfxo_manager/src/sl_hfxo_manager.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.1.3/platform/service/hfxo_manager/src/sl_hfxo_manager_hal_s2.o: C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2/platform/service/hfxo_manager/src/sl_hfxo_manager_hal_s2.c gecko_sdk_4.1.3/platform/service/hfxo_manager/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DEFR32MG21A020F1024IM32=1' '-DSL_APP_PROPERTIES=1' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DCORTEXM3=1' '-DCORTEXM3_EFM32_MICRO=1' '-DCORTEXM3_EFR32=1' '-DPHY_RAIL=1' '-DPLATFORM_HEADER="platform-header.h"' '-DSL_LEGACY_HAL_ENABLE_WATCHDOG=1' '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' '-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' '-DCUSTOM_TOKEN_HEADER="sl_token_manager_af_token_header.h"' '-DUSE_NVM3=1' '-DUC_BUILD=1' '-DSL_ZIGBEE_LEAF_STACK=1' '-DEMBER_MULTI_NETWORK_STRIPPED=1' '-DSL_ZIGBEE_PHY_SELECT_STACK_SUPPORT=1' '-DCONFIGURATION_HEADER="app/framework/util/config.h"' -I"C:\Users\HaraldLIngebrigtsen\SimplicityStudio\Strombergs\strombergs_node_test\config" -I"C:\Users\HaraldLIngebrigtsen\SimplicityStudio\Strombergs\strombergs_node_test\autogen" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/Device/SiliconLabs/EFR32MG21/Include" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/common/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/bootloader" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/bootloader/api" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/cli/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/cli/src" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/CMSIS/Core/Include" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/driver/debug/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/device_init/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/emdrv/common/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/emlib/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/emdrv/gpiointerrupt/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/hfxo_manager/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/iostream/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/legacy_hal/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/legacy_hal_wdog/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/legacy_printf/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/sl_mbedtls_support/config" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/sl_mbedtls_support/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/mbedtls/include" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/mbedtls/library" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/emdrv/nvm3/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/power_manager/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/printf" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/printf/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/sl_psa_driver/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/sl_psa_driver/inc/public" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/common" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/protocol/ble" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/protocol/ieee802154" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/protocol/zwave" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/chip/efr32/efr32xg2x" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin/rail_util_ieee802154" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin/pa-conversions" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin/pa-conversions/efr32xg21" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin/rail_util_pti" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin/rail_util_rf_path" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/se_manager/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/third_party/crypto/sl_component/se_manager/src" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/silicon_labs/silabs_core/memory_manager" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/common/toolchain/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/system/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/sleeptimer/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/service/token_manager/inc" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/platform/micro" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/include" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/common" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/em260" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/util/serial" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/service-function" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/counters" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/framework" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/debug-print" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/network-steering" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/scan-dispatch" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/signature-decode" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/rail_lib/plugin" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/gp" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/zigbee" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/mac/rail_mux" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//platform/radio/mac" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//util/silicon_labs/silabs_core" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/core" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/stack/mac" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/update-tc-link-key" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/include" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/util" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/security" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/util/zigbee-framework" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/util/counters" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/cli" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/util/common" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/util/security" -I"C:/Users/HaraldLIngebrigtsen/SimplicityStudio/SDKs/gecko_sdk_2//protocol/zigbee/app/framework/plugin/ota-storage-common" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse -fno-builtin-printf -fno-builtin-sprintf -Wno-unused-parameter -Wno-missing-field-initializers -Wno-missing-braces -Wno-format -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.1.3/platform/service/hfxo_manager/src/sl_hfxo_manager_hal_s2.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


