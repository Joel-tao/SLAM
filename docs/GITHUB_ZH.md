# GitHub 仓库与后续更新

项目仓库：https://github.com/Joel-tao/SLAM

仓库包含代码、配置、测试、中文说明、演示动画和实验报告。虚拟环境、Python缓存及临时实验输出不纳入版本控制；原始CV和联系方式不在项目中。

下载与运行：

```bash
git clone https://github.com/Joel-tao/SLAM.git
cd SLAM
python3 -m venv .demo-venv
source .demo-venv/bin/activate
pip install -r requirements-lock.txt
python -m construction_slam.demo --gif
python -m unittest discover -s tests -v
```

建议使用Python 3.11。修改后，检查 `git diff` 和测试结果，再执行：

```bash
git add README.md config construction_slam docs tests ros2
 git commit -m "Describe your change"
git push origin main
```

README会显示动画。项目保留AI辅助说明及功能边界。目前未指定开源许可证；如需允许他人复用，请选择并加入LICENSE。
