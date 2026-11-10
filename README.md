# audio_qos

Source code for the WhisPrompt audio evaluation stack:

- simulator/box-sim: acoustic simulation and evaluation utilities.
- firmware/whisprompt-fw: Scepter/Dongle ESP-IDF firmware and PC companion sources.
- hmi/native-console: Windows native speech and microphone application.

Firmware projects require ESP-IDF v5.5.6. Simulator dependencies are listed in simulator/box-sim/requirements.txt. The HMI Native Console uses CMake and the local Windows/Vulkan/CUDA SDKs.

Generated build output, logs, captured audio, test corpora, model weights, local reports, and machine-specific settings are excluded. Runtime datasets and model assets must be supplied separately under their applicable terms.
