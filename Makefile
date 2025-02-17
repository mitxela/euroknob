all : flash

TARGET:=euroknob
ADDITIONAL_C_FILES+= lib_i2c.c

CH32V003FUN:=../ch32v003fun/ch32v003fun
include ../ch32v003fun/ch32v003fun/ch32v003fun.mk

flash : cv_flash
clean : cv_clean

