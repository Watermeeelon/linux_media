stavix-objs	:= stavix_core.o stavix_cards.o stavix_i2c.o stavix_dvb.o stavix_dma.o

obj-$(CONFIG_DVB_STAVIX) += stavix.o

ccflags-y += -Idrivers/media/tuners
ccflags-y += -Idrivers/media/dvb-core
ccflags-y += -Idrivers/media/dvb-frontends
ccflags-y += -Idrivers/media/dvb-frontends/stid135

