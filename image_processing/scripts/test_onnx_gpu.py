import onnxruntime as ort

# 查看可用的执行 providers
available_providers = ort.get_available_providers()
print("可用执行 providers:", available_providers)

# 验证CUDA是否可用（需包含'CUDAExecutionProvider'）
if "CUDAExecutionProvider" in available_providers:
    print("✅ ONNX Runtime GPU版配置成功")
else:
    print("❌ 未检测到CUDA支持，请检查：")
    print("1. CUDA Toolkit是否与PyTorch/ONNX Runtime版本匹配")
    print("2. 环境变量CUDA_HOME是否正确配置")
    print("3. ONNX Runtime是否为GPU版本（pip install onnxruntime-gpu）")
