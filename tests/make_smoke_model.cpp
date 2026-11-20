// SPDX-License-Identifier: Apache-2.0
#include <torch/script.h>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  torch::jit::Module model("LeSTASmokeModel");
  model.define(R"JIT(def forward(self, features: Tensor) -> Tensor:
    return features.narrow(1, 0, 1) * 0.0 + 1.0
)JIT");
  model.save(argv[1]);
}
