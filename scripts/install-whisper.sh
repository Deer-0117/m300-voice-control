#!/usr/bin/env sh
# Run on the target Jetson. Never installs or starts any flight service.
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$TASK_ROOT"
WHISPER_REPO=$(python3 -c 'import json;print(json.load(open("third_party/whisper.lock.json"))["git_url"])')
WHISPER_COMMIT=$(python3 -c 'import json;print(json.load(open("third_party/whisper.lock.json"))["commit"])')
MODEL_URL=$(python3 -c 'import json;print(json.load(open("third_party/whisper.lock.json"))["model_url"])')
MODEL_SHA=$(python3 -c 'import json;print(json.load(open("third_party/whisper.lock.json"))["model_sha256"])')
if [ ! -d third_party/whisper.cpp/.git ]; then
    git clone "$WHISPER_REPO" third_party/whisper.cpp
fi
# This is a dedicated dependency checkout inside the NEW migration folder.
git -C third_party/whisper.cpp checkout --detach "$WHISPER_COMMIT"
cmake -S third_party/whisper.cpp -B third_party/whisper.cpp/build -DWHISPER_BUILD_TESTS=OFF
cmake --build third_party/whisper.cpp/build --target whisper-cli -j2
if [ ! -f third_party/whisper.cpp/models/ggml-base.bin ]; then
    curl --fail --location "$MODEL_URL" -o third_party/whisper.cpp/models/ggml-base.bin
fi
printf '%s  %s\n' "$MODEL_SHA" third_party/whisper.cpp/models/ggml-base.bin | sha256sum --check -
