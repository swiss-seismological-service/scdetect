check_defined = \
    $(strip $(foreach 1,$(1), \
        $(call __check_defined,$(1),$(strip $(value 2)))))
__check_defined = \
    $(if $(value $(1)),, \
      $(error Undefined $(1)$(if $(2), ($(2)))))

$(call check_defined, BUILD_DIR, build directory)
$(call check_defined, SEISCOMP_ROOT, SeisComP root directory)

PATH_APP_DATA=data/app

TEMPLATES_JSON=templates.json
CATALOG=catalog.scml
WAVEFORM_DATA_MSEED=data.mseed
WAVEFORM_DATA_SDS=data.sds

10MIN=10
30MIN=30
FLAG_STARTTIME_10MIN=--starttime 2020-10-25T19:30:00
FLAG_ENDTIME_10MIN=--endtime 2020-10-25T19:40:00
FLAG_STARTTIME_30MIN=--starttime 2020-10-25T19:20:00
FLAG_ENDTIME_30MIN=--endtime 2020-10-25T19:50:00

50HZ=50
100HZ=100
200HZ=200

PERF_50HZ:=$(addsuffix hz, $(addprefix $(PATH_APP_DATA)/perf-, $(50HZ)))
PERF_100HZ:=$(addsuffix hz, $(addprefix $(PATH_APP_DATA)/perf-, $(100HZ)))
PERF_200HZ:=$(addsuffix hz, $(addprefix $(PATH_APP_DATA)/perf-, $(200HZ)))

TEMPLATES_JSON_50HZ:=$(shell find $(PERF_50HZ) -type f -name '$(TEMPLATES_JSON)')
PERF_CONFIGS_50HZ:=$(dir $(TEMPLATES_JSON_50HZ))
TEMPLATES_JSON_100HZ:=$(shell find $(PERF_100HZ) -type f -name '$(TEMPLATES_JSON)')
PERF_CONFIGS_100HZ:=$(dir $(TEMPLATES_JSON_100HZ))
TEMPLATES_JSON_200HZ:=$(shell find $(PERF_200HZ) -type f -name '$(TEMPLATES_JSON)')
PERF_CONFIGS_200HZ:=$(dir $(TEMPLATES_JSON_200HZ))

PERF_CONFIGS:=$(PERF_CONFIGS_50HZ) $(PERF_CONFIGS_100HZ) $(PERF_CONFIGS_200HZ)

CATALOGS:=$(addsuffix $(CATALOG), $(PERF_CONFIGS))

WAVEFORMS_50HZ:=$(addsuffix data.$(10MIN).mseed, $(PERF_CONFIGS_50HZ)) \
								$(addsuffix data.$(30MIN).mseed, $(PERF_CONFIGS_50HZ)) 
WAVEFORMS_100HZ:=$(addsuffix data.$(10MIN).mseed, $(PERF_CONFIGS_100HZ)) \
								 $(addsuffix data.$(30MIN).mseed, $(PERF_CONFIGS_100HZ)) 
WAVEFORMS_200HZ:=$(addsuffix data.$(10MIN).mseed, $(PERF_CONFIGS_200HZ)) \
								 $(addsuffix data.$(30MIN).mseed, $(PERF_CONFIGS_200HZ)) 
WAVEFORMS:=$(WAVEFORMS_50HZ) $(WAVEFORMS_100HZ) $(WAVEFORMS_200HZ)

SDS_ARCHIVE:=$(addprefix $(PATH_APP_DATA)/, $(WAVEFORM_DATA_SDS))

.PHONY: clean
clean:
	rm -rvf $(CATALOGS) $(WAVEFORMS) $(SDS_ARCHIVE)

.PHONY: config
config: $(CATALOGS) $(WAVEFORMS)


%/$(WAVEFORM_DATA_SDS): %/$(WAVEFORM_DATA_MSEED)
	mkdir -p $@
	$(SEISCOMP_ROOT)/bin/scart -I "file://$(realpath $<)" $@

%/$(CATALOG): $(addprefix $(PATH_APP_DATA)/, $(CATALOG))
	cp $< $@

# kill the circular-dependency
$(addprefix $(PATH_APP_DATA)/, $(CATALOG)): ;

duration = $(word 2,$(subst ., ,$(notdir $(1))))MIN
frequency = $(subst hz,,$(word 2,$(subst -, ,$(word 3,$(subst /, ,$(dir $(1)))))))

define prepare_waveform_data =
./prepare_waveform_data.py \
	--target-frequency $(call frequency,$@) \
	$(FLAG_STARTTIME_$(call duration,$@)) \
	$(FLAG_ENDTIME_$(call duration,$@)) \
	--output $(notdir $@) \
	--binary-scmssort $(realpath $(SEISCOMP_ROOT)/bin/scmssort) \
	--binary-prepare \
	$(realpath $(BUILD_DIR)/bin/perf_util_scdetect_cc_prepare_waveform_data) \
	"sdsarchive://$(realpath $(word 2,$^))" $(word 1,$^)
endef

%/data.$(10MIN).mseed: %/$(TEMPLATES_JSON) $(SDS_ARCHIVE)
	$(prepare_waveform_data)

%/data.$(30MIN).mseed: %/$(TEMPLATES_JSON) $(SDS_ARCHIVE)
	$(prepare_waveform_data)
