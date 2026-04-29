
#include <torch/torch.h>
#include <iostream>

int main() {
    torch::Tensor tensor = torch::eye(3);
    std::cout << "LibTorch is working! Matrix:\n" << tensor << std::endl;
    return 0;
}