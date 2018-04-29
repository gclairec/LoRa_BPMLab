#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#

COMPONENT_ADD_INCLUDEDIRS := . ecies gatts LoRa LoRa/cayennelpp LoRa/cayennelpp/include LoRa/common LoRa/common/include LoRa/htu21d LoRa/htu21d/include LoRa/lmic LoRa/lmic/include LoRa/ssd1306 LoRa/ssd1306/include
COMPONENT_SRCDIRS := $(COMPONENT_ADD_INCLUDEDIRS)