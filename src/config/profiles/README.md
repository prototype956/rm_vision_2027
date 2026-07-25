# YAML Profiles

这些文件是基于 src/config/vision.yaml 的覆盖模板，不是完整配置。

使用方式：

```bash
./build/src/test/mv-real-debug-vision --config src/config/profiles/infantry.yaml
./build/src/test/mv-real-debug-vision --config src/config/profiles/hero.yaml
```

程序启动时会先加载基础的 src/config/vision.yaml，再叠加这里的 profile。
程序退出时，Foxglove 热调后的参数会回写到你本次指定的 profile 文件。

建议：

- 通用参数放在 src/config/vision.yaml
- 车种差异参数放在 profiles/*.yaml
- 每台实车可再复制出自己的文件，例如 profiles/infantry_3.yaml
