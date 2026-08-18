把 libtrackseg.so 放这里。

模型权重(seg_best.pth)已经在 PC 上折叠+固化进这个 .so 了(gen/weights_embed.c),
**没有单独的权重文件要拷**, 运行时也不读任何模型文件。

Nano 上:
    cd trackseg_cuda/            # 就是 trackseg_cuda.zip 解出来的工程
    export PATH=/usr/local/cuda/bin:$PATH
    make                         # -> libtrackseg.so (约 3MB, 权重在里面)
    cp libtrackseg.so <本目录>

重训模型后(结构不变, 只换权重):
    PC:   python3 export_cuda_weights.py 新的seg_best.pth   # 重生成 gen/
    Nano: make && cp libtrackseg.so <本目录>

没有 GPU 想先验证管线, 可以用 CPU 版(慢, 约 0.3s/帧):
    make cpu && cp libtrackseg_cpu.so <本目录>
    roslaunch 时加 _trackseg_lib:=<本目录>/libtrackseg_cpu.so
