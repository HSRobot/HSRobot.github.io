# 引言
通过该学习该指南可以了解开发HIROP平台vision模块的基础流程

# 准备
在开始前，我们需要熟悉和了解以下概念。

## 接口
在HIROP vision模块中，将开发接口分为了用户接口和开发者接口。
### 用户接口
用户接口是提供给想使用VISION识别功能开发机器人应用程序的开发者使用的。

### 开发者接口
开发者接口则是为那些想将自己或其他视觉算法集成至vision中算法开发者和集成者使用的。
而本文主要就是描述如何将一个视觉算法集成至vision中。

## 模块
为了集成视觉算法至VISION模块中，我们还需要了解以下概念：训练器和识别器。在集成相关算法时，实际上就是编写这两个模块。

### 训练器
由于VISION模块可以适配使用机器学习及其他需要进行训练的视觉算法，因此VISION模块中独立分出来了训练器的概念：用户算法训练功能实现的模块。
如果使用的算法不需要进行训练，那么则无需为算法编写训练器。

### 识别器
识别器就是用户算法进行实际物体识别的模块，所有的用户算法都必须实现改模块。

## 开发语言
Vision模块支持使用Python和C++两种语言来编写相关算法的实例。不管使用何种语言其开发者接口都是一致的。

---
# 开始
通过了解了上述概念后我们就开始进入我们的开发中了。现在假设我们有一物体识别算法A，其识别之前需要进行训练。因此我们需要开发两个模块：A训练器和A识别器。
下面我将分别讲解C++和Python如何编写算法实例。

---
## C++

### 编写训练器
	
```C++
// 开发者接口源文件
#include <vision/c_base_trainer.h>

class ATrainer : public CBaseTrainer{

public:
	ATrainer();

    /**
     * @brief   A训练器的具体训练实现函数
     * @return  0 训练成功 -1 训练失败
     */
    int train(){
		/**
		 *	具体训练的实现
		 */
	}

    /**
     * @brief   解析由创建传递过来的算法的配置参数
     * @param
     * @return  0 成功 -1 失败
     */
    int parseConfig(const YAML::Node &node){
		/**
		 *	解析参数，比如获取训练的次数，数据集的的路径等
		 */
	}

    /**
     * @brief   获取训练进度
     * @param
     * @return  0 > 获取失败 >=0 进度
     */
    int feedback(){
		/**
		 *	获取并返回当前的训练进度
		 */
	}

    /**
     * @brief   保存训练结果
     * @param   保存路径
     * @return  0 成功 -1 失败
     */
    int saveData(std::string path){
		/**
		 *	保存相关训练的结果，保存在path路径下
		 */
	}

}
// 定义该类为一个插件，继承自训练器
H_DECLARE_PLUGIN(ITrainer)
H_EXPORT_PLUGIN(ATrainer, "ATrainer", "1.0")

```
!!! Note

    在编写算法实例时，我们的训练器实例需要继承与`ITrainer`接口。但是在这里我们却继承了`CBaseTrainer`？这是为什么呢？
    主要是因为Vision模块中为训练器提供了辅助类，帮助我们实现了部分简单的`ITrainer`接口，降低开发者的开发难度。

在ATrainer中，我需要实现：

* `int train()`：该函数就是主要进行物体训练的接口了，算法的具体训练功能在该接口中（阻塞）实现。
* `int parseConfig()`：对于需要一些参数的训练器而言，该函数主要是用于从Vision模块中传递相关的训练器参数的，传递是Yaml对象。对于无需训练参数的算法实例，则无需实现。
* `int feedback()`：该函数主要的目的在于向Vision模块反馈当前训练的进度，如果想为你的训练器提供进度获取的功能，则可在该函数中具体实现获取进度并返回，如不提供进度功能，则无需实现。
* `int saveData()`：由于当期VISION模块中不提供数据保存相关的接口，这就意味着开发者需要自己编写代码来保存相关的训练数据。不过Vison为了方便管理训练数据，会为每个训练器提供一个专有的目录。因此，训练器需要将所有的训练数据保存至指定路径path中。



实现上面的四个接口后，我们的Vision便可以识别和使用ATrainer了。不过在最后，我们需要声明，我们这个类是一个插件。`因为在Vision中，所有的算法实例都是以插件的形式存在`
```C++
	H_DECLARE_PLUGIN(ITrainer)
	H_EXPORT_PLUGIN(ATrainer, "ATrainer", "1.0")
```
对于该宏的使用和定义可参考[HPluin开发指南]()。

### 训练器接口调用顺序
对于上述ATrainer实现的接口，Vision模块对训练接口的调用时机如下。

1. Vision模块首先根据用户请求，调用`ATrainer`构造函数
2. 然后调用`parseConfig()`向ATrainer传递训练的信息，如：物体名称、物体模型路径、数据集路径等
3. 在通过parseConfig解析完相关参数后，vision模块就会调用`train()`接口来进行训练
4. 当通过train()训练完毕后，Vision模块便会调用`saveData()`接口来通知ATrainer进行数据的保存


### 编写识别器
```C++

#include <vision/c_base_detector.h>
class ADetector : public CBaseDetector{

    /**
     * @brief   实现具体的识别功能
     * @return  0 成功 -1 失败
     */
    int detection(){
		/**
		 *	编写具体的识别的代码
		 */
	}

    /**
     * @brief   加载相关的训练结果
     * @param   [objectName] 需要识别的物体
     * @return  0 成功 -1 失败
     */
    int loadData(const std::string path, const std::string objectName){
		/**
		 *	编写从path路径下加载相关的训练文件，如果有的话。具体如何加载是和相关训练器约定好的
		 */
	}

    /**
     * @brief   获取识别的结果
     * @param[out] poses， 保存识别的结果
     * @return  0 成功 -1 失败
     */
    int getResult(std::vector<pose> &poses){
		/**
		 *	获取具体的识别结果，返回的是位姿
		 */
	}

    /**
     * @brief   向检测器传递图像数据
     * @param   [inputImg] 输入，传递的图像
     * @return  void
     */
    void setDepthImg(const cv::Mat &inputImg){
		/**
		 *	编写保存相关深度图的代码
		 */
	}

    /**
     * @brief   向检测器传递图像数据
     * @param   [inputImg] 输入，传递的图像
     * @return  void
     */
    void setColorImg(const cv::Mat &inputImg){
		/**
		 *	编写保存相关彩色图的代码
		 */
	}

	H_DECLARE_PLUGIN(IDetector)
	H_EXPORT_PLUGIN(ADetector, "ADetector", "1.0")

}

```
在编写相关算法实例的识别器的时候，我们的识别器实例需要继承`IDetector`，而此处继承了CBaseDetector，具体原因也是和训练器说的一致。
	
在ADetector中，我们需要实现以下接口

* `int detection()`：该接口主要就是实现具体的物体识别逻辑，该接口会由Vision模块调用
* `int loadData()`：该接口的主要目的就是为了加载被识别物体的训练参数。而相关参数就是保存在path路径下的。而具体的数据的保存格式则是ATrainer定义的。
* `int getResult()`：该接口的主要作用就是当识别结束后，Vision调用该接口来获取具体的识别结果。应次在该结果中实现识别结果的返回。
* `void setDepthImg()`：该接口的主要作用就是保存由Vision模块传递给识别器的深度图像数据，供识别的时候使用，如果识别器识别不需要深度图，则该接口无需实现。
* `void setColorImg()`：该接口的主要作用就是保存由Vision模块传递给识别器的彩色图像数据，供识别的时候使用，如果识别器识别不需要彩色图，则该接口无需实现。

同样的，在实现完上诉接口后，我们需要导出我们的识别器
```C++
	H_DECLARE_PLUGIN(IDetector)
	H_EXPORT_PLUGIN(ADetector, "ADetector", "1.0")
```
对于该宏的使用和定义可参考[HPluin开发指南]()。

### 训练器接口调用顺序
对于上述ADetector实现的接口，Vision模块对其接口的调用时机如下。
	
1. Vision模块首先根据用户请求，调用`ADetector()`构造函数
2. 然后调用ADetector中的`loadData()`接口，来通知ADetector来加载被识别物体的训练参数
3. 在加载完相关参数后，Vision模块便会通过`setDepthImg()`和`setColorImg()`接口向ADetector传递待识别的图像数据
4. 在传递完相关参数后，Vision模块便会调用ADetector的`detection()`接口来进行物体识别
5. 当通过detection()接口执行完成，即识别完成后，Vision模块就会调用`getResult()`接口来获取识别的结果

### 编译
使用下面的Cmake文件来进行工程配置
```cmake
cmake_minimum_required (VERSION 2.6)
project (VisionDemo)

add_library(ADetector ADetector.cpp)
add_library(ATrainer ATrainer.cpp)

find_package(hirop_vision REQUIRED)
INCLUDE_DIRECTORIES(${hirop_vision_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(ADetector ${hirop_vision_LIBRARIES})
TARGET_LINK_LIBRARIES(ATrainer ${hirop_vision_LIBRARIES})
```

```bash
$ cmake ./
$ make
$ export VISION_PLUGIN_PATH=$VISION_PLUGIN_PATH:\`pwd\`
```

### 测试
可使用以下指令进行测试
```bash
$ rosrun vision_bridge vision_bridge
$ rosservice call /detection cokecan A 0 "" 
```
其中第二条指令`$ rosservice call /detection cokecan A 0 "" `

* `cokecan`: 待识别物体名称
* `A`: 识别器名称
* `0`: 识别器的类别， 0为C++ 1为python

---
## Python
对于Python开发者，所有的接口的和C++版本均一致，只是将编程语言变成了Python。下面我们只以编写识别器来作为例子讲解即可。

### 编写识别器
```python

class ADetector:

	def __init__(self):
		# do some init 

	def detection(self):
		# do something detection

	def loadData(self, path, objectname):
		# load data from path

	def getResult(self, poses):
		# returen result

	def setDepthImg(self, img):
		# save depth image

	def setColorImg(self, img):
		# save color image
```

实现一个Python版本的识别器只需实现一个拥有上述接口的类即可。其作用和调用顺序和C++版本的一致

### 编译
无需编译，只需将python源文件命名为ADetector.py并放置在VISION_PLUGIN_PATH环境变量路径下即可
	
### 测试
```bash
$ rosrun vision_bridge vision_bridge
$ rosservice call /detection cokecan A 1 "" 
```
