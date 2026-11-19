import torch, sys
print("python", sys.version.split()[0])
print("torch", torch.__version__)
print("cuda available", torch.cuda.is_available())
if torch.cuda.is_available():
    print("gpu", torch.cuda.get_device_name(0))
    print("capability", ".".join(map(str, torch.cuda.get_device_capability(0))))
    x = torch.randn(4096, 4096, device="cuda")
    y = (x @ x.T).sum().item()
    print("matmul ok", round(y, 2))