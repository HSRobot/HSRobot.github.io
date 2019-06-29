# HPlugin插件开发指南
HPlugin是一个轻量级的插件模块，可依赖HPlugin框架迅速为工程实现插件化。

## 编译安装
```bash
$ mkdir build; cd build
$ qmake ./
$ make ; make install
```

## 制作插件

### 声明插件接口
在接口的定义头文件中使用H_DECLARE_INTERFACE宏来声明一个接口。
```C++
class ISimaplePlugin{

public:
    virtual int SayHi(std::string name) = 0;

};
H_DECLARE_INTERFACE(ISimaplePlugin,"ISimaplePlugin/v1.0")
```
!!! Note

    如果接口处于某个命名空间中，那么该宏应当在命名空间外部使用。
    H_DECLARE_INTERFACE(namespace::ISimaplePlugin,"ISimaplePlugin/v1.0")

### 声明插件
插件都是继承于接口的，假如我们有插件A，其类名为`SimaplePluginA`，其继承于`ISimaplePlugin`。
那么在定义插件类的头文件中，我们需要使用`H_DECLARE_PLUGIN`宏来声明一个插件，其中宏的参数是表明该插件式基于哪个接口的。
```C++
class SimaplePluginA:public ISimaplePlugin{
public:
    SimaplePluginA();
    int SayHi(std::string name);
};

H_DECLARE_PLUGIN(ISimaplePlugin)
```

### 导出插件
经过上面的两步，定义接口和定义插件。那么接下来，我们就需要给我们定义的插件进行实现和导出，只有导出后的插件，才能被插件系统所加载。
因此，在插件的实现文件中，我们应当使用`H_EXPORT_PLUGIN`宏来导出一个插件。
```C++
SimaplePluginA::SimaplePluginA(){

    std::cout << "SimaplePluginA init" << std::endl;

}

int SimaplePluginA::SayHi(std::string name){

    std::cout << "SimaplePluginA say = " << name << " lol" << std::endl;
    return 0;
}

H_EXPORT_PLUGIN(SimaplePluginA, "SimaplePluginA", "1.0")
```
其中宏`H_EXPORT_PLUGIN`有三个参数
 * 声明要导出的插件类
 * 声明导出插件的名称 '需要和上一个参数一致'
 * 声明导出插件的版本

### 工程配置
!!! Note

    在工程配置中，应当注意生成的插件动态库的名称为"lib"+插件类名+".so"。如：插件名为`SimaplePluginA`，则其动态库名称为`libSimaplePluginA.so`

## 调用插件

### 插件加载器
为了加载插件，我们需要在我们的使用插件系统的程序上，获取我们的插件加载器。
```C++
int main(){
	HPluginLoader *loader
	loader = HPluginLoader::getLoader();
}
```
通过HPluginLoader::getLoader()函数，来获取当前系统的插件加载器指针。

在加载完插件加载器后，我们需要为插件加载器指定，插件的加载路径，而这一动作我们通过
```C++
loader->setPath("./");
```
这一函数来指定，如果传递的路径为空，那么插件加载器便会通过LD_LIBRARY_PATH环境变量来搜索相关的库。

### 加载插件
获取完插件加载器和设置相关路径后，便可以通过插件加载器来加载相关的插件了。我们通过一个Hplugin来描述一个插件。
```C++
	HPlugin *p;
	    p = loader->load("SimaplePluginA");

    if(!p){
        cout << "load plugin error" << endl;
        return -1;
    }
```
当我们通过load函数加载插件失败时，它将会返回一个空指针。开发者很有必要对该值进行判断。

### 实例化插件
在我们通过上面几步后，我们拿到了一个插件对象Hplgun，但是这个插件对象中并不是我们真正的插件类的指针。
我们还需要将其实例化后，才能拿到插件的指针。
```C++
	ISimaplePlugin *pluginPtr;
	pluginPtr = p->instance<ISimaplePlugin>();
	
	if(!pluginPtr){
        cout << "instance plugin error" << endl;
        return -1;
    }
```
我们将通过`instance`函数来获取一个插件的实例指针，它是一个模板函数，需要指明要实例化的插件接口类。通过模板函数来防止开发者将错误的实例赋值给插件。
如果通过instance获取实例化插件失败（类型错误或其他原因）时，将会返回一个空指针。

我们通过`instance`获取到插件实例后，便可以使用该实例来使用插件的功能了。
```C++
	pluginPtr->SayHi("FSHS");
```
当使用完插件后，我们需要将插件进行卸载。
```C++
	loader->unload(p);
```
