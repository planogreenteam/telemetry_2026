REPO_ROOT := $(abspath ../..)
PYINSTALLER := python3 -m PyInstaller

.PHONY: all receiver sender clean

all: receiver sender

receiver:
	cd $(REPO_ROOT) && $(PYINSTALLER) \
		--clean \
		--onefile \
		--name telemetry_receiver \
		--paths "$(REPO_ROOT)" \
		--hidden-import BMS.bms_handler \
		--hidden-import BMV.bmv_handler \
		--hidden-import CAN.can_handler \
		--hidden-import LORA.radio_config \
		--hidden-import LORA.lora_transport \
		--hidden-import RTD.rtd_handler \
		--hidden-import storage.csv_sink \
		--hidden-import storage.event_csv_sink \
		--hidden-import telemetry_packet \
		telemetry_receiver.py

sender:
	cd $(REPO_ROOT) && $(PYINSTALLER) \
		--clean \
		--onefile \
		--name telemetry_sender \
		--paths "$(REPO_ROOT)" \
		--hidden-import BMS.bms_normalizer \
		--hidden-import BMS.bms_reader \
		--hidden-import BMV.bmv_normalizer \
		--hidden-import BMV.bmv_policy \
		--hidden-import BMV.bmv_reader \
		--hidden-import CAN.can_normalizer \
		--hidden-import CAN.can_reader \
		--hidden-import LORA.radio_config \
		--hidden-import LORA.lora_transport \
		--hidden-import RTD.max31865_reader \
		--hidden-import RTD.rtd_normalizer \
		--hidden-import spidev \
		--hidden-import storage.csv_sink \
		--hidden-import telemetry_packet \
		--hidden-import transmit_policy \
		telemetry_sender.py

clean:
	rm -rf $(REPO_ROOT)/build $(REPO_ROOT)/dist
