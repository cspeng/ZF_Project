```

```

# C++

###### 1.C语言和C++有什么区别？

面向对象语言，继承派生、多态、虚函数、内联、函数重载

模板、STL。

一、主体不同

1）C语言是面向过程的、抽象化的通用程序设计语言，广泛应用于底层开发。

2）C++:是C语言的继承，既可以进行过程化程序设计，又可以进行以抽象数据类型的特点基于对象的程序设计

二、优势不同

1）C语言：以简便的方式编译、处理低级存储器。产生少量的机器语言以及不要运行环境支持便能运行的高效率程序设计语言。

2）C++：高效运行的实用性特征，提高大规模程序的编程质量与程序设计语言的问题描述能力。

三、特点不同

1）C语言：保持着跨平台的特性

2）C++：类是支持数据封装的工具，对象则是数据封装的实现。C++通过建立用户定义类支持数据封装和数据隐藏。

###### 2.struct和class有什么区别？

在C++中：

1）默认继承权限不同，class继承默认是private继承，而struct继承是Public继承

2）class 还可以用于定义模板参数，像typename,但是关键字struct不能用于定义模板参数，不可用于声明模板

```c++
template<class T>
class test{};
template<struct T>//报错，非类型模板参数
struct MyStruct{};
```



###### 3.extern "C"的作用？

extern

1同一个工程一个c文件函数调用另一个c文件函数
2 c文件调用另一个c文件变量
3 c++调用c函数
4 c 调用c++函数

extern "C"的主要作用就是为了能够正确实现C++代码调用其他C语言代码。加上extern "C"后，会指示编译器这部分代码按C语言（而不是C++）的方式进行编译。由于C++支持函数重载，因此编译器编译函数的过程中会将函数的参数类型也加到编译后的代码中，而不仅仅是函数名；而C语言并不支持函数重载，因此编译C语言代码的函数时不会带上函数的参数类型，一般只包括函数名

```c++
#ifndef __INCvxWorksh /*防止该头文件被重复引用*/
#define __INCvxWorksh
#ifdef __cplusplus             //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"{
    #include "xxxx.h"
#endif
 
/*…*/
 
#ifdef __cplusplus
}
 
#endif
#endif /*end of __INCvxWorksh*
```

###### 4.函数重载和覆盖有什么区别？

重载：函数重载是指在同一作用域内（名字空间），可以有一组具有相同函数名，不同参数列表的函数。

覆盖（也叫重写）：指在派生类中重新对基类中的虚函数（注意是虚函数）重新实现。即函数名和参数都一样。

隐藏：指派生类中的函数把基类中相同名字的函数屏蔽掉了。和上面两个的区别在多态的实现上。

```c++
存在如下声明：
void f (); //全局函数
class A
{
public:
 void f(int);
};

class B: public A
{
public:
 void f(int *);
 static void f(int **);
 void test();
 //...
};

那么在如下B::test实现中哪几个函数调用是合法的：
void B::test()
{
 int x = 0;
 int *p = NULL;
 f();    //(1)
 f(x);   //(2)
 f(&x);  //(3)
 f(&p);  //(4)
};
A.(1)(2)(3)(4) B.(1)(3)(4) C.(2)(3)(4) D.(3)(4)
//答案是D，类成员函数重载：局部同名函数将隐藏而不是重载全局声明，不引入父类名字空间时子类的同名函数不会和父类的构成重载，静态成员函数可以和非静态成员函数构成重载。
```

###### 5.谈一谈你对多态的理解，运行时多态的实现原理是什么？

C++ 默认构造函数是对类中的参数提供默认值的构造函数，一般情况下，是一个没有参数值的空函数，也可以提供一些的默认值的构造函数，如果用户没有定义构造函数，那么编译器会给类提供一个默认的构造函数，**但是只要用户自定义了任意一个构造函数，那么编译器就不会提供默认的构造函数**，这种情况下，容易编译报错，所以正确的写法就是用户在定义构造函数的时候，也需要添加一个默认的构造函数，这样就不会造成编译报错


多态就是将基类类型的指针或者引用指向派生类型的对象。多态通过虚函数机制实现。

多态的作用是接口重用。

1.指相同对象收到不同消息或不同对象收到相同消息产生不同的实现操作。

a.编译时多态性：通过重载函数实现

b.运行时多态性：通过虚函数实现

2.虚函数

​	虚函数是在基类中声明为virtual,并在派生类中重新定义的成员函数，可实现成员函数的动态重载

3.抽象类

​	包含纯虚函数的类称为抽象类。由于抽象类包含了没有定义的纯虚函数，所以不能定义抽象类的对象。

**01虚函数**

- 在类的定义中，前面有 virtual 关键字的成员函数称为虚函数；

- virtual 关键字只用在类定义里的函数声明中，写函数体时不用。

  ```c++
  
  class Base 
  {
      virtual int Fun() ; // 虚函数
  };
   
  int Base::Fun() // virtual 字段不用在函数体时定义
  { }
  ```

  

**02多态表现形式一**

1）派生类的指针可以赋给基类指针

2）通过基类指针调用基类和派生类中的同名[虚函数]时：

若该指针指向一个基类的对象，那么被调用是基类的虚函数；

若该指针指向一个派生类的对象，那么被调用的是派生类的虚函数。

**这种机制就叫做多态，调用哪个虚函数，取决于指针对象指向哪种类型的对象。**

```c++
// 基类
class CFather 
{
public:
    virtual void Fun() { } // 虚函数
};
 
// 派生类
class CSon : public CFather 
{ 
public :
    virtual void Fun() { }
};
 
int main() 
{
    CSon son;
    CFather *p = &son;
    p->Fun(); //调用哪个虚函数取决于 p 指向哪种类型的对象
    return 0;
}
// p 指针对象指向的是 CSon 类对象，所以 p->Fun() 调用的是 CSon 类里的 Fun 成员函数
```

**03多态表现形式二**

- 派生类的对象可以赋给基类[引用]

- 通过基类引用调用基类和派生类中的同名[虚函数]时

  1）若该引用引用的是一个基类的对象，那么被调用是基类的虚函数

  2）若该引用引用的是一个派生类的对象，那么被调用的是派生类的虚函数

  这种机制也叫做多态，**调用哪个虚函数，取决于引用的对象是哪种类型的对象。**

  ```c++
  // 基类
  class CFather 
  {
  public:
      virtual void Fun() { } // 虚函数
  };
   
  // 派生类
  class CSon : public CFather 
  { 
  public :
      virtual void Fun() { }
  };
   
  int main() 
  {
      CSon son;
      CFather &r = son;
      r.Fun(); //调用哪个虚函数取决于 r 引用哪种类型的对象
      return 0;
  }
  //r 引用的对象是 CSon 类对象，所以 r.Fun() 调用的是 CSon 类里的 Fun 成员函数。
  ```

  **04多态的简单示例**

```c++
class A 
{
public :
    virtual void Print() { cout << "A::Print"<<endl ; }
};
 
// 继承A类
class B: public A 
{
public :
    virtual void Print() { cout << "B::Print" <<endl; }
};
 
// 继承A类
class D: public A 
{
public:
    virtual void Print() { cout << "D::Print" << endl ; }
};
 
// 继承B类
class E: public B 
{
    virtual void Print() { cout << "E::Print" << endl ; }
};


int main() 
{
    A a; B b; E e; D d;
    
    A * pa = &a; 
    B * pb = &b;
    D * pd = &d; 
    E * pe = &e;
    
    pa->Print();  // a.Print()被调用，输出：A::Print
    
    pa = pb;
    pa -> Print(); // b.Print()被调用，输出：B::Print
    
    pa = pd;
    pa -> Print(); // d.Print()被调用，输出：D::Print
    
    pa = pe;
    pa -> Print(); // e.Print()被调用，输出：E::Print
    
    return 0;
}
```

**05 多态的作用**

在面向对象的程序设计中使用多态，能增强程序的可扩充性，即程序需要修改或增加功能的时候，需要**改动和增加的代码较少**。

**在非构造函数，非析构函数的成员函数中调用「虚函数」，是多态!!!**

在构造函数和析构函数中调用「虚函数」，不是多态。编译时即可确定，调用的函数是**自己的类或基类**中定义的函数，不会等到运行时才决定调用自己的还是派生类的函数。

```c++
// 基类
class CFather 
{
public:
    virtual void hello() // 虚函数
    {
        cout<<"hello from father"<<endl; 
    }
    
    virtual void bye() // 虚函数
    {
        cout<<"bye from father"<<endl; 
    }
};
 
// 派生类
class CSon : public CFather
{ 
public:
    CSon() // 构造函数
    { 
        hello(); 
    }
    
    ~CSon()  // 析构函数
    { 
        bye();
    }
 
    virtual void hello() // 虚函数
    { 
        cout<<"hello from son"<<endl;
    }
};
 
int main()
{
    CSon son;
    CFather *pfather;
    pfather = & son;
    pfather->hello(); //多态
    return 0;
}

hello from son  // 构造son对象时执行的构造函数
hello from son  // 多态
bye from father // son对象析构时，由于CSon类没有bye成员函数，所以调用了基类的bye成员函数

```

多态的实现原理

多态的关键在于通过基类指针或引用调用一个虚函数时，编译时不能缺定调用的是基类还是派生类的函数，运行时才能确定。

```c++
class A 
{
public:
    int i;
    virtual void Print() { } // 虚函数
};
 
class B
{
public:
    int n;
    void Print() { } 
};
 
int main() 
{
    cout << sizeof(A) << ","<< sizeof(B);
    return 0;
}
//在64位机子，执行的结果
//16,4
//可以发现有虚函数的类，多出了 8 个字节，在 64 位机子上指针类型大小正好是 8 个字节，这多出 8 个字节的指针有什么作用呢？
```

**01 虚函数表**

每一个有「虚函数」的类（或有虚函数的类的派生类）都有一个「虚函数表」，该类的任何对象中都放着**虚函数表的指针**。「虚函数表」中列出了该类的「虚函数」地址。

**多出来的 8 个字节就是用来放「虚函数表」的地址。**

```c++

// 基类
class Base 
{
public:
    int i;
    virtual void Print() { } // 虚函数
};
 
// 派生类
class Derived : public Base
{
public:
    int n;
    virtual void Print() { } // 虚函数
};
```

上面 Derived 类继承了 Base类，两个类都有「虚函数」，那么它「虚函数表」的形式可以理解成下图：

![image-20210624020916567](C:\Users\10594\AppData\Roaming\Typora\typora-user-images\image-20210624020916567.png)

虚函数表指针走向图

多态的函数调用语句被编译成一系列根据基类指针所指向的（或基类引用所引用的）对象中**存放的虚函数表的地址**，在虚函数表中查找虚函数地址，并调用虚函数的指令。

**02 证明虚函数表指针的作用**

在上面我们用 `sizeof` 运算符计算了有虚函数的类的大小，发现是多出了 8 字节大小（64位系统），这多出来的 8 个字节就是指向「虚函数表的指针」。「虚函数表」中列出了该类的「虚函数」地址。

```c++
// 基类
class A 
{
public: 
    virtual void Func()  // 虚函数
    { 
        cout << "A::Func" << endl; 
    }
};
 
// 派生类
class B : public A 
{
public: 
    virtual void Func()  // 虚函数
    { 
        cout << "B::Func" << endl;
    }
};
 
int main() 
{
    A a;
    
    A * pa = new B();
    pa->Func(); // 多态
    
    // 64位程序指针为8字节
    int * p1 = (int *) & a;
    int * p2 = (int *) pa;
    
    * p2 = * p1;
    pa->Func();
    
    return 0;
}
//输出结果
//B::Func
//A::Func
```

- 第 25-26 行代码中的 `pa` 指针指向的是 `B` 类对象，所以 `pa->Func()` 调用的是 `B` 类对象的虚函数 `Func()`，输出内容是 `B::Func` ；
- 第 29-30 行代码的目的是把 `A` 类的头 8 个字节的「虚函数表指针」存放到 `p1` 指针和把 `B` 类的头 8 个字节的「虚函数表指针」存放到 `p2` 指针；
- 第 32 行代码目的是把 `A` 类的「虚函数表指针」 赋值给 `B` 类的「虚函数表指针」，所以相当于把 `B` 类的「虚函数表指针」 替换 成了 `A` 类的「虚函数表指针」；
- 由于第 32 行的作用，把 `B` 类的「虚函数表指针」 替换 成了 `A` 类的「虚函数表指针」，所以第 33 行调用的是 `A` 类的虚函数 `Func()`，输出内容是 `A::Func`

通过上述的代码和讲解，可以有效的证明了「虚函数表的指针」的作用，「虚函数表的指针」指向的是「虚函数表」，「虚函数表」里存放的是类里的「虚函数」地址，那么在调用过程中，就能实现多态的特性。

**虚析构函数**

析构函数是在删除对象或退出程序的时候，自动调用的函数，其目的是做一些资源释放。

那么在多态的情景下，通过基类的指针删除派生类对象时，通常情况下只调用基类的析构函数，这就会存在派生类对象的析构函数没有调用到，存在资源泄露的情况。

```c++
// 基类
class A 
{
public: 
    A()  // 构造函数
    {
        cout << "construct A" << endl;
    }
    
    ~A() // 析构函数
    {
        cout << "Destructor A" << endl;
    }
};
 
// 派生类
class B : public A 
{
public: 
    B()  // 构造函数
    {
        cout << "construct B" << endl;
    }
    
    ~B()// 析构函数
    {
        cout << "Destructor B" << endl;
    }
};
 
int main() 
{
    A *pa = new B();
    delete pa;
    
    return 0;
}
//construct A
//construct B
//Destructor A
```

从上面的输出结果可以看到，在删除 `pa`指针对象时，`B` 类的析构函数没有被调用。

**解决办法：把基类的析构函数声明为virtual**

- 派生类的析构函数可以 virtual 不进行声明；
- 通过基类的指针删除派生类对象时，首先调用派生类的析构函数，然后调用基类的析构函数，还是遵循「先构造，后虚构」的规则。

```c++
// 基类
class A 
{
public: 
    A()  
    {
        cout << "construct A" << endl;
    }
    
    virtual ~A() // 虚析构函数
    {
        cout << "Destructor A" << endl;
    }
};
//construct A
//construct B
//Destructor B
//Destructor A
```

所以要养成好习惯:

- 一个类如果定义了虚函数，则应该将析构函数也定义成虚函数;
- 或者，一个类打算作为基类使用，也应该将析构函数定义成虚函数。
- 注意：不允许构造函数不能定义成虚构造函数

**纯虚函数和抽象类**

纯虚函数： 没有函数体的虚函数

```c++
class A 
{
 
public:
    virtual void Print( ) = 0 ; //纯虚函数
private: 
    int a;
};

A a;         // 错，A 是抽象类，不能创建对象
A * pa ;     // ok,可以定义抽象类的指针和引用
pa = new A ; // 错误, A 是抽象类，不能创建对象
```

包含纯虚函数的类叫抽象类

- 抽象类只能作为基类来派生新类使用，不能创建抽象类的对象
- 抽象类的指针和引用可以指向由抽象类派生出来的类的对象

实现多态的三个条件：1）要有继承；2）要有虚函数重写；3）要有父类指针（引用）指向子类对象。

多态的原理：动态连编，根据实际对象的类型还判断重写函数的调用。



###### 6.对虚函数机制的理解，单继承、多继承、虚继承条件下虚函数表的结构

在类的定义中，前面有virtual关键字的成员函数称为虚函数。

如果有虚函数表，那么只有一个虚函数表，并且按照虚函数声明的顺序顺序排列，派生类的虚函数紧接着基类的虚函数排列。

多继承中会有多个虚函数表，几重继承就会有几个虚函数表。这些虚函数表会按照派生的顺序依次排列。如果子类改写了父类的虚函数，那么就会用子类自己的虚函数覆盖相应的父类虚函数；如果子类有新的虚函数，那么就添加到第一个虚函数表的末尾。

![image-20210624115332084](C:\Users\10594\AppData\Roaming\Typora\typora-user-images\image-20210624115332084.png)

[](https://www.cnblogs.com/ziolo/archive/2013/05/07/3066022.html)

```c++
class A
{
public:
  A()
  {
    m_nData = 1;
  }
  virtual void fun()
  {

  }
  virtual void fun1()  // 新增加
  {
    
  }
  int m_nData;
};

class B
{
public:
  B()
  {
    m_nData = 2;
  }
  virtual void fun()
  {
    
  }
  virtual void fun2()  // 新增加
  {
    
  }
  int m_nData;
};

class AB :public A, public B
{
public:
  AB()
  {
    m_nData = 3;
  }
  virtual void fun()
  {
    
  }
  virtual void fun3()  // 新增加
  {
    
  }
  int m_nData;
};

  
int main(int argc, char* argv[])
{
  AB the;

  return 0;
}
```

1、A::Vtable如下：

&AB::fun  &A::fun1  AB::fun3

2、B::Vtable如下:

&AB::fun  &B::fun2

先按继承声明顺序依次构造虚表，如果子类有虚函数，并且不同名，则填写到声明顺序首位的基类虚表中的末尾项。

###### 7.如果虚函数是有效的，那为什么不把所有函数设为虚函数？

不行。首先，虚函数是有代价的，由于每个虚函数的对象都要维护一个虚函数表，因此在使用虚函数的时候都会产生一定的系统开销，这是没有必要的。

###### 8.构造函数可以是虚函数吗？析构函数可以是虚函数吗？

虚函数的调用需要虚函数表指针，而该指针存放在对象的内存空间中；若构造函数声明为虚函数，那么由于对象还未创建，还没有内存空间，更没有虚函数表地址用来调用虚函数——构造函数了。

构造一个对象的时候，必须知道对象的实际类型，而虚函数行为是在运行期间确定实际类型的。
而在构造一个对象时，由于对象还未构造成功。编译器无法知道对象 的实际类型，是该类本身，
还是该类的一个派生类，或是更深层次的派生类。无法确定。

虚函数的执行依赖于虚函数表。而虚函数表在构造函数中进行初始化工作，即初始化vptr，
让他指向正确的虚函数表。
而在构造对象期间，虚函数表还没有被初 始化，将无法进行。

 首先析构函数可以为虚函数，而且当要使用基类指针或引用调用子类时，最好将基类的析构函数声明为虚函数，否则可能存在内存泄露的问题。

###### 9.什么场景需要用到纯虚函数？纯虚函数的作用是什么？

一定义：

纯虚函数是一种特殊的虚函数，一般格式为

class<类名>
{

​	virtual <类型><函数名>（<参数表>）=0；

};

作用：实现留给基类的派生类去做。

二、引入原因

方便使用多态特性。

基类本身不需要生成对象。含有纯虚函数的类称为抽象类。

为派生类提供一个一致的接口。

###### 10.了解RAII吗？介绍一下？

Resource Acquisition is Initialization,资源获取即初始化，将资源的生命周期与一个对象的生命周期绑定，举例来说，把一些资源封装在类中，在构造函数请求资源，在析构函数中释放资源且绝不抛出异常，而一个对象在生命周期结束时会自动调用析构函数，即资源的生命周期与一个对象的生命周期绑定。

###### 11.类的大小计算。

[](https://blog.csdn.net/fengxinlinux/article/details/72836199)

如果用sizeof运算符对一个类型名操作，得到的是具有该类型实体的大小。

这里指的类的大小，是指类的对象所占的大小。

关于类/对象大小的计算

- 首先，类大小的计算遵循结构体的对齐原则
- 类的大小与普通数据成员有关，与成员函数和静态成员无关。即普通成员函数，静态成员函数、静态数据成员、静态常量数据成员均对类的大小无影响。
- 虚函数对类的大小有影响，是因为虚函数表指针带来的影响
- 虚继承对类的大小有影响，，是因为虚基表指针带来的影响
- 空类的大小是一个特殊情况，空类的大小为1

解释说明

- 静态数据成员之所以不计算在类的对象大小内，是因为类的静态数据成员被该类所有的对象共享，并不属于具体哪个对象，静态数据成员定义在内存的全局区。

64位gcc编译器，指针大小为8；

一、简单情况的计算

```c++
#include<iostream>
using namespace std;


class base
{
    public:
    base()=default;
    ~base()=default;
    private:
    static int a;
    int b;
    char c;

};


int main()
{
    base obj;
    cout<<sizeof(obj)<<endl;
}
```

计算结果：8，由于字节对齐，4+4=8；

二、空类的大小

```c++
#include <iostream>
using namespace std;

class NoMembers
{
};

int main()
{
    NoMembers n;  // Object of type NoMembers.
    cout << "The size of an object of empty class is: "
         << sizeof(n) << endl;

```

new需要分配不同的内存地址，不能分配内存大小为0的空间。

注意两种情况

```c++
class Empty {};
struct D : public Empty { int a;};
//派生类继承空类后，空基类的一个字节不会加到派生类中
//sizeof(D)为4；
```

```c++
class Empty {};
class HoldsAnInt {
    int x;
    Empty e;
};
//sizeof(HoldsAnInt)为8
//由于字节对齐的原因
```

三、含有虚函数成员

虚函数是通过一张虚函数表（virtual table）来实现的，**编译器必须保证虚函数表的指针**位于对象实例中最前面的位置（为了保证正确取到虚函数的偏移量）。

每当创建一个包含有虚函数的类或从包含有虚函数的类派生一个类时，编译器就会为这个类创建一个**虚函数表（VTABLE）保存该类所有虚函数的地址**，其实这个VTABLE的作用就是保存自己类中所有虚函数的地址，可以把VTABLE形象地看成一个函数指针数组，这个数组的每个元素存放的就是虚函数的地址。在每个带有虚函数的类 中，编译器秘密地置入一指针，称为**v p o i n t e r（缩写为V P T R），指向这个对象的V TA B L E。** 当构造该派生类对象时，其成员VPTR被初始化指向该派生类的VTABLE。所以可以认为VTABLE是该类的所有对象共有的，**在定义该类时被初始化**；**而VPTR则是每个类对象都有独立一份的**，且在该类对象被构造时被初始化。

四、虚继承的情况

对虚继承层次的对象的内存布局，在不同编译器实现有所区别。 
在这里，我们只说一下在gcc编译器下，虚继承大小的计算。

在gcc下，不管是否虚继承，GCC都是将**虚表指针在整个继承关系中共享**的，不共享的是指向虚基类的指针。

```c++
class A {

    int a;

};

class B:virtual public A{

    virtual void myfunB(){}

};

class C:virtual public A{

    virtual void myfunC(){}

};

class D:public B,public C{

    virtual void myfunD(){}

};
```

以上代码中sizeof(A)=16,sizeof(B)=24,sizeof(C)=24,sizeof(D)=32. 
解释：A的大小为int大小加上虚表指针大小。B，C中由于是虚继承因此大小为int大小加指向虚基类的指针的大小。B,C虽然加入了自己的虚函数，但是虚表指针是和基类共享的，因此不会有自己的虚表指针，他们两个共用虚基类A的虚表指针。D由于B,C都是虚继承，因此D只包含一个A的副本，于是D大小就等于int变量的大小+B中的指向虚基类的指针+C中的指向虚基类的指针+一个虚表指针的大小,由于字节对齐，结果为8+8+8+8=32

###### 12.volatile关键字的作用？

```c++
int a = 100;

while (a == 100) { 
// code
}
```

编译时，编译器发现没有企图改变a的值，它可能会进行优化，变成while(true)的死循环，但时有时可能会产生没有预期的行为，为了产生预期的行为，需要阻止编译器做这种优化，可以用volatile关键字修饰

```c++
volatile int a = 100;
```

而volatile关键字告诉编译器其修饰的变量是易变的，同理编译器根据易变属性也会做一些操作。它会确保修饰的变量每次都读操作都从内存里读取，每次写操作都将值写到内存里。

volatile 修饰结构体时，结构体的成员也是volatile的？

是的，都是。

```c++
struct A{int data};
volatile A a;
const A b;
```

volatile可以保证原子性？

```c++
volatile int i =0;
i++;

//先读取i的值到tmp
//增加tmp的值
//把tmp的值写回到i的地址里
```

而volatile只能保证内存可见，可以理解为上述三步中的每一步都是原子的，但是三步合起来却不一定是原子的，因为在多线程中三步中间可能插入一些其它操作改变了预期的行为，所以volatile不能用在多线程中，多线程中的原子操作还是需要使用atomic。单例模式的double check方法中instance变量为什么需要使用volatile修饰也是这个原理。

volatile不能解决多线程安全问题，

- std::atomic用于多线程访问的数据，且不用互斥量，用于并发编程中

- volatile用于读写操作不可以被优化掉的内存，用于特种内存中

std::atomic对int,char,bool等数据结构进行原子性封装，在多线程环境中，对std::atomic对象的访问不会造成竞争-冒险，利用 std::atomic可实现数据结构的无锁设计

13.如何实现线程池

[](https://mp.weixin.qq.com/s?__biz=MzkyODE5NjU2Mw==&mid=2247484800&idx=1&sn=402ac9441890f201a5eb46641307a832&chksm=c21d373cf56abe2aa51c2aa20314a395c86266ae72a0e53fdcbc00d07d9d180308b183a48be0&scene=21#wechat_redirect)

- 核心线程数（core_threads）：线程池中拥有的最少线程个数，初始化时就会创建好的线程，常驻于线程池。
- 最大线程个数（max_threads)：线程池中拥有的最大线程个数。
- max_threads>=core_threads,当任务的个数太多线程池执行不过来时，内部就会创建更多的线程用于执行更多的任务，内部线程数不会超过max_threads,多创建出来的线程在一段时间内没有执行任务则会自动被回收掉，最终线程个数保持在核心线程数。
- 超时时间（time_out)：多创建出来的线程在time_out时间内没有执行任务就会被回收。
- 可获取当前线程池中线程的总个数
- 可获取当前线程池中空闲线程的个数
- 开启线程池功能的开关
- 关闭线程池功能的开关，可以选择是否立即关闭，立即关闭线程池，当前线程池里缓存的任务不会被执行。

###### 14.了解各种强制类型转换的原理及使用？

1.static_cast:用于非多态类型转换（静态转换），不能用于两个不相关的类型

原有的自动类型转换，例如short转int、int转double、向上转型等。

void指针和具体类型指针之间的转换，例如void * 转int  * 、 char*转void *等；

不能用于无关类型之间的转换，两个具体类型之间的转换，例如int * 转double *  ，int 和指针之间的转换。

1）用于类层次结构中基类和派生类之间指针或引用的转换

进行上行转换（把派生类的指针或引用转换成基类表示）是安全的
      进行下行转换（把基类的指针或引用转换为派生类表示），由于没有动态类型检查，所以是不安全的

2）用于基本数据类型之间的转换，如把int转换成char。这种转换的安全也要开发人员来保证

3）把空指针转换成目标类型的空指针

4）把任何类型的表达式转换为void类型

static_cast不能转换掉expression的const、volitale或者__unaligned属性。

```c++
class Complex{
public:
    Complex(double real = 0.0, double imag = 0.0): m_real(real), m_imag(imag){ }
public:
    operator double() const { return m_real; }  //类型转换函数
private:
    double m_real;
    double m_imag;
};
int main(){
    //下面是正确的用法
    int m = 100;
    Complex c(12.5, 23.8);
    long n = static_cast<long>(m);  //宽转换，没有信息丢失
    char ch = static_cast<char>(m);  //窄转换，可能会丢失信息
    int *p1 = static_cast<int*>( malloc(10 * sizeof(int)) );  //将void指针转换为具体类型指针
    void *p2 = static_cast<void*>(p1);  //将具体类型指针，转换为void指针
    double real= static_cast<double>(c);  //调用类型转换函数
   
    //下面的用法是错误的
    float *p3 = static_cast<float*>(p1);  //不能在两个具体类型的指针之间进行转换
    p3 = static_cast<float*>(0X2DF9);  //不能将整数转换为指针类型
    return 0;
}
```

2.reinterpret_cast:将一种类型转换为另一种不同的类型

用法：reinterpret_cast<type_id> (expression)
    type-id必须是一个指针、引用、算术类型、函数指针或者成员指针。

对二进制位的重新解释，简单粗暴，对static_cast的一种补充，例如两个具体类型指针之间的转换、int和指针之间的转换

```c++
char str[] = "hello";
float*p1 = reinterpret_cast<float*>(str)
int *p = reinterpret_cast<int*>(100);
p = reinterpret_cast<int*>(new A(25,96));
```

3.const_cast:用于const与非const、volatile与非volatile之间的转换

而是去除指向常数对象的指针或引用的常量性，其去除常量性的对象必须为指针或引用。

```c++
const int n = 100;
int *p = const_cast<int*>(&n);
*p = 234;
 cout<<"n = "<<n<<endl;
 cout<<"*p = "<<*p<<endl;

//n = 100
//*p = 234
```



4.dynamic_cast：用于将一个父类对象的指针转换为子类对象的指针或引用。（动态交换）

1）其他三种都是编译时完成的，dynamic_cast是运行时处理的，运行时要进行类型检查

2）不能用于内置的基本数据类型的强制转换。

3）dynamic_cast转换如果成功的话返回的是指向类的指针或引用，转换失败的话则会返回NULL。

4）使用dynamic_cast进行转换的，基类中一定要有虚函数，否则编译不通过。

        B中需要检测有虚函数的原因：类中存在虚函数，就说明它有想要让基类指针或引用指向派生类对象的情况，此时转换才有意义。



**产生这种运行期的错误原因在于static_cast强制类型转换时并不具有保证类型安全的功能，而C++提供的dynamic_cast却能解决这一问题，dynamic_cast可以在程序运行时检测类型转换是否类型安全。**

**当然dynamic_cast使用起来也是有条件的，它要求所转换的操作数必须包含多态类类型（即至少包含一个虚函数的类）。**

向上转型：子类对象指针--》父类对象指针/引用（不需要转化）

向下转型：父类对象指针--》子类对象指针/引用（用dynamic_cast转型是安全的）

要借助RTTI进行检测。

只能转换指针类型或者引用类型。

向上转型不执行运行期检测，提高效率，留下安全隐患。

向下转型会借助RTTI信息进行安全检测。

注意：

dynamic_cast:只能用于有虚函数的类

dynamic_cast：会先检查能否转型成功，能成功则转型，不能成功则返回0

dynamic_cast 会在程序运行过程中遍历继承链，如果途中遇到了要转换的目标类型，那么就能够转换成功，如果直到继承链的顶点（最顶层的基类）还没有遇到要转换的目标类型，那么就转换失败。对于同一个指针（例如 pa），它指向的对象不同，会导致遍历继承链的起点不一样，途中能够匹配到的类型也不一样，所以相同的类型转换产生了不同的结果。

###### 15.指针和引用有什么区别？什么情况下用指针，什么情况下用引用？

引用在c++内部是指针常量，如int &ref = int *const ref

引用只是取得数据，无权修改，指针可以修改指向

引用访问一个变量是直接访问，而指针是简介访问

引用是一个变量的别名，本身不单独分配自己的内存空间，而指针有自己的内存空间。

引用必须初始化，而指针不用

1）需要改变实参的时候，只能用指针

2）传递大型结构并且“只读”其元素的时候

3）动态分配空间时，必须用指针

4）传递数组时，必须用指针

5）函数返回指针时

6）如果变量指向一个对象，并且该变量不为空，此时可声明为引用。

7）如果使用一个变量并让它指向一个对象，但是该变量在某些时候也可能不指向任何对象，这时应该把变量声明为指针

8）数据对象较大则使用const指针或const引用，以提高效率

###### 16.一般什么情况下会出现内存泄漏？怎么用C++在编码层面尽量避免内存泄漏。

当我们用new在堆上创建变量后，没有调用delete函数。

1）不要手动管理内存，可以尝试在适用的情况下使用智能指针

2）使用string而不是char*,string类在内部处理所有内存管理，而且速度快且优化好

3）不要使用原始指针，除非用旧的lib接口

4）new和delete要成对出现，培养良好的编码习惯

5）任何需要动态内存的东西应该隐藏在一个RAII对象中，当它超出范围时释放内存，RAII在构造函数中分配内存并在析构函数中释放内存

###### 17.unique_ptr如何转换所有权？

使用std::move()将对象的所有权转移到另一个unique_ptr.

```c++
std::unique_ptr<int>p1(new int(42))
std::unique_ptr<int>p2 = std::move(p1);
```



###### 18.谈一谈你对面向对象的理解

C++面向对象的三大特性为：==封装、继承、多态==

在现实生活中的任何物体都可以归为一类事物，而每一个个体都是一类事物的实例。面向对象的编程是以对象为中心，以消息为驱动，所以程序=对象+消息。

封装就是将一类事物的属性和行为抽象成一个类，使其属性私有化，行为公开化，提高了数据的隐秘性的同时，使代码模块化。这样做使得代码的复用性更高。

继承则是进一步将一类事物共有的属性和行为抽象成一个父类，而每一个子类是一个特殊的父类--有父类的行为和属性，也有自己特有的行为和属性。这样做扩展了已存在的代码块，进一步提高了代码的复用性。

如果说封装和继承是为了使代码重用，那么多态则是为了实现接口重用。多态的一大作用就是为了解耦--为了解除父子类继承的耦合度。

万物皆对象，对象上有其属性和行为

人可以作为对象，属性有姓名、年龄、身高、体重...，行为有走、跑、跳、吃饭、唱歌...

具有相同性质的==对象==，我们可以抽象称为==类==，人属于人类，车属于车类

###### 19.什么场景下使用继承方式，什么场景下使用组合？

区别：继承是子类对父类的发展，组合是一个类把另一个类当做组件来用

组合：需要保证类的安全性，不破坏封装，一个类只作为另一个类的组件来用

继承：两个类具有发展关系。

1.父类只是给子类提供服务，并不涉及子类的业务逻辑

2.层级关系明确，功能划分清晰，父类和子类各做各的。

3.父类的所有变化，都需要在子类中体现，耦合成为需求

当类之间有显著的不同，并且较小的类是组成较大类所需要的组件。

当类之间有很大相似的功能，可以提取这些共同的功能做成基类。

###### 20.**new和malloc有什么区别？**

1).malloc与free是c++/c语言的标准函数，new/delete是c++的运算符

2).他们都可用于申请动态内存和释放内存。new/delete比malloc/free更加智能，其实底层也是执行的malloc/free。为啥说new/delete更加的智能？因为new和delete在对象创建的时候自动执行构造函数，对象消亡之前会自动执行析构函数。

3）new返回指定类型的指针，并且可以自动计算出所需要的大小

malloc必须用户指定大小，并且默然返回类型为void*,必须强行转换为实际类型的指针

什么是堆：堆是大家共有的空间，分全局堆和局部堆。全局堆就是所有没有分配的空间，局部堆就是用户分配的空间。堆在操作系统对进程 初始化的时候分配，运行过程中也可以向系统要额外的堆，但是记得用完了要还给操作系统，要不然就是内存泄漏。

栈是线程独有的，保存其运行状态和局部自动变量的。栈在线程开始的时候初始化，每个线程的栈互相独立。每个函数都有自己的栈，栈被用来在函数之间传递参数。操作系统在切换线程的时候会自动的切换栈，就是切换SS/ESP寄存器。栈空间不需要在高级语言里面显式的分配和释放。

free()释放的是指针指向的内存！注意！释放的是内存，不是指针！这点非常非常重要！指针是一个变量，只有程序结束时才被销毁。释放了内存空间后，原来指向这块空间的指针还是存在！只不过现在指针指向的内容的垃圾，是未定义的，所以说是垃圾。因此，前面我已经说过了，释放内存后把指针指向NULL，防止指针在后面不小心又被解引用了。


- malloc的内存可以用delete释放吗？

会出现 内存泄漏、内存释放不完全等等

用malloc 和free 的话 不能调用到 构造函数 和析构函数

- malloc出来20字节内存，为什么free不需要传入20呢，不会产生内存泄漏吗？

  在malloc时，所分配的不仅是你请求的那点空间，还加了一个信息块来记录额外信息，这个信息块位于你请求的空间前面。而malloc返回指针的指向的是你请求的空间。你向malloc要了999字节，如果malloc分配的最小粒度是1024字节，那么你得到的就是1024字节。

- new[]和delete[]一定要配对使用吗？new[]和delete[]为何要配对使用？

new[]会在K * N个空间的基础上，头部多申请4个字节，用于存储数组长度，这样delete[]时候才知道对象数组的大小，才会相应调用K次析构函数，并且释放K*N+4大小的内存。

delete不同于delete[]，它认为这只是一个对象占用的空间，不是对象数组，不会访问前4个字节获取长度，所以只调用了一次析构函数

不定次数的析构函数，并且挂掉，是因为在new时候没有多申请4个字节存储长度，而delete[]时候还会向前找4个字节获取长度，这4个字节是未定义的，所以调用了不固定次数的析构函数，释放内存的时候也释放了起始地址为A-4的内存，而正常的起始地址应该是A，所以程序挂掉。

int是内置类型，new[]和delete[]在配合int使用时知道int是内置类型，不需要析构函数，所以也就不需要多4个字节来存放数组长度，只需要直接操作内存即可。

当类型为int, float等内置类型时，new、delete、new[]、delete[]不需要配对使用；

当是自定义类型时，new、delete和new[]、delete[]才需要配对使用。

###### 21.**C++11新特性你都了解多少？**

- 了解auto和decltype吗？

auto可以让编译器在编译时就推导出变量的类型，

```c++
auto a = 10; //10是int型 ，可以自动推导出a是int

```

```c++
int i =10;
auto a = i,&b = i,*c = &i;//atuo相当于int
auto d =0, f = 1.0; //error,0和1.0类型不同，对于编译器有二义性，没法推导
auto e;//error,使用auto必须马上初始化
```

```c++
void func(auto value){} //error,auto 不能作函数参数
class A{
    auto a = 1;//error, 在类中auto不能用作非静态成员
    static auto b=1;//error,与auto无关，正常static int b = 1也不可以，要在类外定义和初始化
    static const auto int c = 1;//ok
};

void func2(){
    int a[10] = {0};
    auto b = a;//ok
    auto c[10] = a; //error,auto 不能定义数组，可以定义指针
    vector<int>d;
    vector<auto>f = d;//error,无法推导出模板参数
}
```

- auto的使用必须马上初始化，否则无法推导出类型
- auto在一行定义多个变量时，各个变量的推导不能产生二义性，否则编译失败
- auto不能用作函数参数
- 在类中auto不能用作非静态成员变量
- auto不能定义数组，可以定义指针
- auto无法推导出模板参数

```c++
int i=0;
auto *a = &i; //int *
auto &b = i; //int &;
auto c = b; //int ,忽略 了引用

const auto d  =i;//const int 
auto e = d; //int ,忽略const

const auto &f = e;//const int &
auto &g=f;// const int &
```

推导规则

- 在不声明为引用或指针时，，auto会忽略等号右边的引用类型和cv限定

- 在声明为引用或者指针时，auto会保留等号右边的引用和cv属性

  复杂类型的时候可以使用auto

  ```c++
  auto func = [&](){
  		cout<<"xxx";
  }; //不关心lambda表达式是什么类型
  auto asyncfunc = std::async(std::launch::async,func);
  ```

  auto用于推导变量类型，而decltype则用于推导表达式类型

  ```c++
  int func(){return 0;}
  decltype(func())i;//i为int类型
  int x = 0;
  decltype(x)y; //y是int类型
  decltype(x+y)z; //z是int类型
  ```

  decltype不会像auto忽略引用和cv属性，会保留表达式的引用和cv属性

  ```c++
  const int&i = 1;
  decltype(i)b=2;//b是const int&
  ```

  对于decltype（exp）有

- exp是表达式，decltype(exp)和exp类型相同

- exp是函数调用，decltype(exp)和函数返回值类型相同

- 其他情况，若exp是左值，decltype(exp)是exp类型的左值引用

  ```c++
  int a =0,b=0;
  decltype(a+b)c=0;//c是int,(a+b)返回一个右值
  decltype(a+=b)d = c;//d是int&,因为，（a+=b)返回一个左值
  d = 20;
  cout<<"c"<<c<<endl;//输出20；
  ```

  auto和decltype一般配合使用在推导函数返回值的类型上面

  ```c++
  template<typename T,typename U>
  return_value add(T t,U u){//t和u类型不确定，无法推导
  	retrun t+U;
  }
  ```

  ```c++
  template<typename T,typename u>
  decltype(t+u)add(T t,U u){ //t和u尚未定义
  	return t+U;
  }
  //代码在C++11编译不过，因为decltype(t+u)推导时，t和u尚未定义
  ```

  ```c++
  template<typename T,typename U>
  auto add(T t,U u)->decltype(t+u){
      return t+u;
  }
  ```

- 返回值后置类型语法就是为了解决函数返回值类型依赖于参数但却难以确定返回值类型的问题
- 谈一谈你对左值和右值的了解，了解左值引用和右值引用吗？

1）左值：非临时性对象的表达式。有名字，可以取地址。如非匿名对象（包含变量），函数返回的引用，const 对象等都是左值。

2）右值：临时对象的表达式，没有名字，临时生成的，不可取地址，如立即数，函数的返回值。

```c++
vector<string>arr(3);
const int x = 2;
int y;
int z = x+y;
string str="foo";
//arr,str,y,z都是左值，x也是一个左值，而类似于2，x+y这类临时（没有专属变量名）的值则是右值。
```

3）左右值替换：

右值转换为左值：直接新建变量然后赋值就可以了。

int b = a+1;将a+1这个右值转变成左值

左值转化为右值：std::move()

move(a):将这个左值转变为了右值

4）左值引用

左值引用本质是指针常量，左值引用只能引用左值

```c++
int a = 10;
int &b = a;//定义一个左值引用变量
b = 20;   //通过左值引用修改引用内存的值
int &b = a;//实践执行的是 int *constb=&a;
```

5）常引用

```c++
int &var = 10;
//无法通过，因为10无法进行取地址操作，无法对一个立即数取地址，立即数并没有在内存中，而是在寄存器，可以通过常引用解决
const int &var = 10;
//此刻在内存上产生了临时变量保存了10，临时变量可以取地址，相当于
//conat int temp = 10;
//const int &var = temp
```

6)右值引用

左值引用只能引用左值，而常引用只能通过引用来读取数据，无法去修改数据，因为被const修饰，所有增加右值引用专门来引用右值

声明：通过两个&&来声明。如：int &&x = 5;

右值引用绑定到右值，绑定后本来会被销毁的右值的生存期会延长至于绑定到它的右值引用的生存期。

在汇编层面右值引用做的事情和常引用一样，既产生临时量来存储常量，但是，右值引用可以进行读写操作，而常引用只能进行读操作。

右值引用的存在并不是为了取代左值引用，而是充分利用右值（特别是临时对象）的构造来减少对像构造和析构操作来提高效率的目的

7）左值引用的用途

1.作为复杂名称变量的别名

```c++
auto &whichList = theList[myHash(x,theList.size())];
```

2.用于自身调用

通过rangeFor循环使一个vector对象所有都增加1，下面的循环是做不到

```c++
for(auto x:arr)  //x仅相当于每个元素拷贝
    ++x;

```

```c++
for(auto &x:arr)
    ++x;
```

3.函数的返回引用

假定有一个findMax函数，它返回一个vector中最大的元素，若给vector存储的是某些大的对象时，下述代码x拷贝返回的最大值到x的内存中

```c++
auto x= findMax(vector);
```

在大型的项目中这显然会增大程序的开销，这时我们可以通过引用来减小这类开销：

```
auto &x = findMaxx(vector);
```

类似的，我们在处理函数返回值的时候也可以使用传引用返回。
但是要注意，当返回的是类中私有属性时，传回的引用会导致外界能够对其修改。

函数返回值时会产生一个临时变量作为函数返回值的副本，而返回引用时不会产生值的副本。

T f(); 返回一般的类类型，返回的类类型不能作为左值，但可以直接调用成员函数来修改，返回类类型调用复制构造函数。
const T f(); 此种类型与上述第一种相同，唯一不同的是返回的类类型不能调用成员函数来修改，因为有const限定符。
T& f(); 返回类的引用可以作为左值，并且返回的类类型引用可以直接调用成员函数来修改，返回的类类型调用移动构造函数。
const T& f(); 不能作为左值，不能调用成员函数修改，不会调用复制构造函数。
4.参与函数中的参数传递

在C和C++的函数中，函数对直接传入的形参进行修改并不会改变实参的值。而有时我们希望能够修改实参，例如经典的Swap()函数。
要想实现修改实数，我们可以通过**传入指针**和**传入引用**实现。

8）右值引用的用途

1.避免拷贝，提高性能，实现move()

右值中的数据可以被安全移走这一特性使得右值被用来表达移动语义。以同类型的右值构造对象时，需要以引用形式传入参数。右值引用顾名思义专门用来引用右值，左值引用和右值引用可以被分别重载，这样确保左值和右值分别调用到拷贝和移动的两种语义实现。对于左值，如果我们明确放弃对其资源的所有权，则可以通过std::move()来将其转为右值引用。std::move（）实际上是对static_cast<T&&>()的简单封装。

2.避免重载参数的复杂性，实现forward()

9）std::move和std::swap

前面我们谈到可以使用引用来减少复制产生的内存开销。

考虑这样一种情况，在进行元素交换时，我们通常使用一个缓存变量temp来临时保存数据；而对temp直接进行=的赋值操作时，实际上temp复制了一次原有对象的内存，但我们需要只是对象之间的移动而不是复制，而C++STL中的std::move函数便可以达成这一操作。

```c++
void swap(vector<string>&x,vector<string>&y){
    vector<string>temp = std::move(x);
    x= std::move(y);
    y = std::move(temp);
}
```

C++引入移动构造函数，专门处理像这种，用a初始化b后，就将a析构的情况。

移动构造函数的参数和拷贝构造函数不同，拷贝构造函数的参数是一个左值引用，但是移动构造函数的初值是一个右值引用。这意味着，移动构造函数参数是一个右值或者将亡值的引用。也就是说，用一个右值或者将亡值初始化另一个对象的时候，才会调用移动构造函数。而move语句，就是将一个左值变成一个将亡值。
移动构造函数应用最多的地方就是STL中。



- 了解移动语义和完美转发吗？

左值、右值
概念1：

左值：可以放到等号左边的东西叫左值。

右值：不可以放到等号左边的东西就叫右值。

概念2：

左值：可以取地址并且有名字的东西就是左值。

右值：不能取地址的没有名字的东西就是右值。

```c++
int a = b+c;
```

a是左值，有变量名，可以取地址，也可以放到等号左边, 表达式b+c的返回值是右值，没有名字且不能取地址，&(b+c)不能通过编译，而且也不能放到等号左边。

```c++
int a  =4;//a是左值，4作为普通字面量是右值。
```

左值一般有：

函数名和变量名

返回左值引用的函数调用

前置自增自减表达式++i、--i

由赋值表达式或赋值运算符连接的表达式(a=b, a += b等)

解引用表达式*p

字符串字面值"abcd"

纯右值、将亡值
纯右值和将亡值都属于右值。

纯右值
运算表达式产生的临时变量、不和对象关联的原始字面量、非引用返回的临时变量、lambda表达式等都是纯右值。

举例：

除字符串字面值外的字面值

返回非引用类型的函数调用

后置自增自减表达式i++、i--

算术表达式(a+b, a*b, a&&b, a==b等)

取地址表达式等(&a)

将亡值
将亡值是指C++11新增的和右值引用相关的表达式，通常指将要被移动的对象、T&&函数的返回值、std::move函数的返回值、转换为T&&类型转换函数的返回值，将亡值可以理解为即将要销毁的值，通过“盗取”其它变量内存空间方式获取的值，在确保其它变量不再被使用或者即将被销毁时，可以避免内存空间的释放和分配，延长变量值的生命周期，常用来完成移动构造或者移动赋值的特殊任务。

```c++
class A{
	xxx
};
A a;
auto c = std::move(a);  //c是将亡值
auto d = static_cast<A&&>(a);//d是将亡值
```

左值引用、右值引用
根据名字大概就可以猜到意思，左值引用就是对左值进行引用的类型，右值引用就是对右值进行引用的类型，他们都是引用，都是对象的一个别名，并不拥有所绑定对象的堆存，所以都必须立即初始化。

```c++
type &name = exp; //左值引用
type &&name = exp;//右值引用
```

左值引用

```c++
int a = 5;
int &b =a;
b =4;
int &c = 10; //error,10无法取地址
const int &d = 10;//c常引用可以，会储存在内存中
```

对于左值引用，等号右边的值必须可以取地址，如果不能取地址，则会编译失败，或者可以使用const引用形式，但这样就只能通过引用来读取输出，不能修改数组，因为是常量引用。

右值引用

如果使用右值引用，那表达式等号右边的值需要时右值，可以使用std::move函数强制把左值转换为右值。

```c++
int a =4;
int &&b = a;//error,a是左值
int &&c= std::move(a); //ok
```

移动语义

谈移动语义前，我们首先需要了解深拷贝与浅拷贝的概念

深拷贝、浅拷贝

```c++
class A{
    public:
    	A(int size):size_(size){
            data_ = new int[size];
        }
    A(){}
    A(const A&a){
		size_= a.size_;
        data_=a.data_;
        cout<<"copy"<<endl;
    }
    ~A(){
        delete[] data_;
    }
    int *data_;
    int size_;
};
int main(){
	A a(10);
     A b = a;
    cout<<"b"<<b.data_<<endl;
    cout<<"a"<<a.data_<<endl;
    return 0;
}
```

上面代码中，两个输出的是相同的地址，a和b的data_指针指向了同一块内存，这就是浅拷贝，只是数据的简单赋值，那再析构时data_内存会被释放两次，导致程序出问题，这里正常会出现double free导致程序崩溃的，如何消除这种隐患呢，可以使用如下深拷贝：

```c++
class A{
    public:
    	A(int size):szie_(size){
            data_=new int[size];
        }
    	A(constA&a){
            size_=a.size_;
            data_ = new int[size_];
            cout<<"copy"<<endl;
        }
    	~A(){
            delete[]data_;
        }
    	int *data_;
    	int size_;
};
int main(){
    A a(10);
    A b = a;
    cout << "b " << b.data_ << endl;
    cout << "a " << a.data_ << endl;
    return 0;
}
```

深拷贝就是再拷贝对象时，如果被拷贝对象内部还有指针引用指向其它资源，自己需要重新开辟一块新内存存储资源，而不是简单的赋值。

移动语义，在程序喵看来可以理解为转移所有权，之前的拷贝是对于别人的资源，自己重新分配一块内存存储复制过来的资源，而对于移动语义，类似于转让或者资源窃取的意思，对于那块资源，转为自己所拥有，**别人不再拥有也不会再使用，**通过C++11新增的移动语义可以省去很多拷贝负担，怎么利用移动语义呢，是通过移动构造函数。

```c++
class A{
    public:
    	A(int size):size_(size){
			data_=new int[size];
        }
    	A(){}
    	A(const A&a){
            size_=a.size_;
            data_=new int[szie_];
            cout<<"copy"<<endl;
        }
    	A(A&&a){
            this->data_= a.data_;
            a.data_=nullptr;
            cout<<"move"<<endl;
        }
    	~A(){
            if(data_!=nullptr){
                delete[]data_;
            }
        }
};
int main(){
    	A a(10);
    	A b = a;
    	A c = std::move(a); //调用移动构造函数
    	return 0;
}
```

如果不使用std::move()，会有很大的拷贝代价，使用移动语义可以避免很多无用的拷贝，提供程序性能，C++所有的STL都实现了移动语义，方便我们使用。例如：

```c++
std::vector<string>vecs;
std::vector<string>vecm = std::move(vecs);//免去很多拷贝
```

**移动语义仅针对于那些实现了移动构造函数的类的对象**，对于那种基本类型int、float等没有任何优化作用，还是会拷贝，因为它们实现没有对应的移动构造函数。

完美转发
完美转发指可以写一个接受任意实参的函数模板，并转发到其它函数，目标函数会收到与转发函数完全相同的实参，转发函数实参是左值那目标函数实参也是左值，转发函数实参是右值那目标函数实参也是右值。那如何实现完美转发呢，答案是使用std::forward()。

```c++
void PrintV(int &t){
    cout<<"lvalue"<<endl;
}
void PrintV(int &&t){
	cout<<"rvalue"<<endl;
}
template<typename T>
void Test(T &&t){
	PrintV(t);
    PrintV(std::forward<T>(t));
    
    PrintV(std::move(t));
}

int main(){
    Test(1); //lvalue rvalue rvalue
    int a = 1;
    Test(a); //lvalue lvalue rvalue
    Test(std::forward<int>(a));  //lvalue rvalue rvalue
    Test(std::forward<int&>(a)); //lvalue lvalue rvalue
    Test(std::forward<int&&>(a)); //lvalue rvalue rvalue
    return 0;
}
```

Test(1)：1是右值，模板中T &&t这种为万能引用，右值1传到Test函数中变成了右值引用，但是调用PrintV()时候，t变成了左值，因为它变成了一个拥有名字的变量，所以打印lvalue，而PrintV(std::forward<T>(t))时候，会进行完美转发，按照原来的类型转发，所以打印rvalue，PrintV(std::move(t))毫无疑问会打印rvalue。

Test(a)：a是左值，模板中T &&这种为万能引用，左值a传到Test函数中变成了左值引用，所以有代码中打印。

Test(std::forward<T>(a))：转发为左值还是右值，依赖于T，T是左值那就转发为左值，T是右值那就转发为右值。



**返回值优化**

返回值优化(RVO)是一种C++编译优化技术，当函数需要返回一个对象实例时候，就会创建一个临时对象并通过复制构造函数将目标对象复制到临时对象，这里有复制构造函数和析构函数会被多余的调用到，有代价，而通过返回值优化，C++标准允许省略调用这些复制构造函数。

那什么时候编译器会进行返回值优化呢?

- return的值类型与函数的返回值类型相同
- return的是一个局部对象

```c++
std::vector<int>return_vector(void){
    std::vector<int>tmp{1,2,3,4,5};
    return tmp;
}
std::vector<int>&&rval_ref = return_vector();

```

不会触发RVO，拷贝构造了一个临时的对象，临时对象的生命周期和rval_ref绑定，等价于下面这段代码：

```
const std::vector<int>&rval_ref = return_vector();
```

```c++
std::vector<int>&&return_vector(void){
	std::vector<int>tmp{1,2,3,4};
    return std::move(tmp);
}
std::vector<int>&&rval_ref=return_vector();
```

这段代码会造成运行时错误，因为rval_ref引用了被析构的tmp。讲道理来说这段代码是错的，但我自己运行过程中却成功了，我没有那么幸运，这里不纠结，继续向下看什么时候会触发RVO。

```c++
std::vector<int>return_vector(void){
	std::vector<int>tmp{1,2,3,4,5};
    return std::move(tmp);
}
std::vector<int>&&rval_ref = return_vector();
```

std::move一个临时对象是没有必要的，也会忽略掉返回值优化。

```c++
std::vector<int>return_vector(void){
	std::vector<int>tmp{1,2,3,4,5};
    return tmp;
}
std::vector<int>rval_ref=return_vector();
```

这段代码会触发RVO，不拷贝也不移动，不生成临时对象。



- 了解列表初始化吗？

  在c++11中可以直接在变量名后面加上初始化列表来进行对象的初始化

  ```c++
  struct A{
      public:
      	A(int){}
      private:
      	A(constA&){}
  };
  int main(){
      A a(123);
      A b = 123; //error
      A c = {123};
      A d{123};//c++11
      
      int e = {123};
      int f{124};//c++11
      return 0;
  }
  ```

  ```c++
  std::vector<int>func(){
  	return {}
  }
  //可用在函数的返回值上
  ```

  列表初始化规则：

  ​	聚合类型可以进行直接列表初始化。

  1.类型是一个普通数组，如int[5],char[],double[]等。

  2.类型是一个类，且满足一下条件：

  - 没有用户声明的构造函数

  - 没有用户提供的构造函数（允许显示预置或弃置的构造函数）

  - 没有私有或保护的非静态数据成员

  - 没有积累

  - 没有虚函数

  - 没有{}和=直接初始化的非静态数据成员

  - 没有默认成员初始化器

    ```c++
    struct A{
      int a;
      int b;
      int c;
        A(int,int){}
    };
    int main(){
    	A a{1,2,3};//error,A有自定义的构造函数，不能列表初始化
    }
    ```

    ```c++
    struct A{
      	int a;
        int b;
        virtual void func(){}//含有虚函数，不是聚合类
    };
    
    struct Base{};
    struct B:public Base{    //有基类，不是聚合类
      int a;
        int b;
    };
    struct C{
      	int a;
        int b = 10;	//有等号初始化，不是聚合类
    };
    struct D{
        int a;
        int b;
        private:
        	int c;//含有私有的非静态数据成员，不是聚合类
    };
    
    struct E{
        int a;
        int b;
        E():a(0),b(0){}//含有默认成员初始化器，不是聚合类
    };
    ```

    对于一个聚合类型，使用列表初始化相当于对其中的每个元素分别赋值；对于非聚合类型，需要先自定义一个对应的构造函数，此时列表初始化将调用相应的构造函数。

  - ```c++
    struct CustomVec{
      	std::vector<int>data;
        CustomVec(std::initializer_list<int>list){
            for(auto iter = list.begin();iter!=list.end();++iter){
               data.push_back(*iter);
            }
        }
    };
    ```

    我想通过上面这段代码大家可能已经知道STL是如何实现的任意长度初始化了吧，这个std::initializer_list其实也可以作为函数参数。

    注意：std::initializer_list<T>，它可以接收任意长度的初始化列表，但是里面必须是相同类型T，或者都可以转换为T。

    列表初始化的好处

    个人认为列表初始化的好处如下：

    方便，且基本上可以替代括号初始化

    可以使用初始化列表接受任意长度

    可以防止类型窄化，避免精度丢失的隐式类型转换

    什么是类型窄化，列表初始化通过禁止下列转换，对隐式转化加以限制：

    - 从浮点类型到整数类型的转换
    - 从 long double 到 double 或 float 的转换，以及从 double 到 float 的转换，除非源是常量表达式且不发生溢出
    - 从整数类型到浮点类型的转换，除非源是其值能完全存储于目标类型的常量表达式
    - 从整数或无作用域枚举类型到不能表示原类型所有值的整数类型的转换，除非源是其值能完全存储于目标类型的常量表达式

  ```c++
  int main(){
      int a = 1,2;//ok
      int b = {1.2};//errror
      
      float c = 1e70;
      float d ={1e70};//errror
      
      float e  = (unsigned long long)-1; //ok
      float f=  {(unsigned long long)-1}; //error
      float g = (unsigned long long )1; //ok
      float h = {(unsigned long long)1};//ok
      
      const int i = 1000;
      const int j =2;
      char k = i;//ok
      char l = {i};//error
      char m = j; //ok
      char m = {j};//ok,因为是const类型，，如果去掉const属性，也会报错；
  }
  ```

  ```c++
  test.cc:24:17: error: narrowing conversion of ‘1.2e+0’ from ‘double’ to ‘int’ inside { } [-Wnarrowing]
      int b = {1.2};
                  ^
  test.cc:27:20: error: narrowing conversion of ‘1.0000000000000001e+70’ from ‘double’ to ‘float’ inside { } [-Wnarrowing]
       float d = {1e70};
  
  test.cc:30:38: error: narrowing conversion of ‘18446744073709551615’ from ‘long long unsigned int’ to ‘float’ inside { } [-Wnarrowing]
      float f = {(unsigned long long)-1};
                                       ^
  test.cc:36:14: warning: overflow in implicit constant conversion [-Woverflow]
      char k = i;
               ^
  test.cc:37:16: error: narrowing conversion of ‘1000’ from ‘int’ to ‘char’ inside { } [-Wnarrowing]
      char l = {i};
  ```

  

- 平时会用到function、bind、lambda吗，都什么场景下会用到？

**std::function**

讲std::function前首先需要了解下什么是可调用对象

满足以下条件之一就可称为可调用对象：

- 是一个函数指针
- 是一个具有operator()成员函数的类对象（传说中的仿函数），lambda表达式
- 是一个可被转换为函数指针的类对象
- 是一个类成员（函数）指针
- bind 表达式或其他函数对象

而std::function就是上面这种**可调用对象的封装器**，可以把std::function看做一个函数对象，用于表示函数这个抽象概念。std::function的实例可以存储、复制和调用任何可调用对象，存储的可调用对象称为std::function的目标，若std::function不含目标，则称它为空，调用空的std::function的目标会抛出std::bad_function_call异常。

```c++
std::function<void(int)>f//这里表示function的对象f的参数是int,返回值是void
    
#include <functional>
#include <iostream>
    
struct Foo{
    Foo(int num):num_(num){}
    void print_add(int i)const{std::cout<<num_+i<<'\n';}
    int num_;
};

void print_num(int i){std::cout<<i<<'\n';}
struct PrintNum{
    void operator()(int i)const {std::cout<<i<<'\n';}
};

int main(){
    //存储自由函数
    std::function<void(int)>f_display = print_num;
    f_display(-9);
    
    //存储lambda
    std::function<void()>f_display_42 = [](){print_num(42);};
    f_display_42();
    //存储到std::bind调用的结果
    std::function<void()>f_display_31337 = std::bind(print_num,31337);
    f_display_31337();
    
    //存储到成员函数的调用
    std::function<void(const Foo&,int)>f_add_display = &Foo::print_add;
    const Foo foo(314159);
    f_add_display(foo,1);
    f_add_display(314159,1);
    
    //存储到数据成员访问 器的调用
    std::function<int(Foo const&)>f_num=&Foo::num_;
    std::cout<<"num_:"<<f_num(foo)<<'\n';
    
    //存储到成员函数及对象的调用
    using std::placeholders::_1;
    std::function<void(int)>f_add_display2 = std::bind(&Foo::print_add,foo,_1);
    f_add_display2(2);
    //存储到成员函数和对象指针的调用
    std::function<void(int)>f_add_display3 = std::bind(&Foo::print_add,&foo,_1);
    f_add_display3(3);
    
    //存储到函数对象的调用
    std::function<void(int)>f_display_obj = PrintNum();
    f_display_obj(18);
}

```

从上面可以看到std::function的使用方法，当给std::function填入合适的参数表和返回值后，它就变成了可以容纳所有这一类调用方式的函数封装器。std::function还可以用作回调函数，或者在C++里如果需要使用回调那就一定要使用std::function，特别方便，这方面的使用方式大家可以读下我之前写的关于线程池和定时器相关的文章。

std::bind

使用std::bind可以将可调用对象和参数一起绑定，绑定后的结果使用std::function进行保存，并延迟调用到任何我们需要的时候。

std::bind通常有两大作用：

- 将可调用对象与参数一起绑定为另一个std::function供调用
- 将n元可调用对象转成m(m < n)元可调用对象，绑定一部分参数，这里需要使用std::placeholders

```c++
#include <functional>
#include <iostream>
#include <memory>

void f(int n1,int n2, int n3,const int&n4,int n5){
    std::cout<<n1<<' '<<n2 <<' '<<n3<<' '<<n4<<' '<<n5<<std::endl;
}

int g(int n1){return n1;}

struct Foo{
    void print_num(int n1,int n2){std::cout<<n1+n2<<std::endl;}
    int data = 10;
};

int main(){
    using namespace std::placeholders;//针对_1,_2,_3.。。
    //演示参数重排序和按引用传递
    int n = 7;
    //(_1与_2来自std::placeholders,并表示将来会传递给f1的参数)
    auto f1 = std::bind(f,_2,42,_1,std::cref(n),n);
    n = 10;
    f1(1,2,1001);  //1为_1所绑定，2为_2所绑定，不使用1001
    				//进行到f(2,42,1,n,7)的调用
    //嵌套bind子表达式共享占位符
    auto f2 = std::bind(f,_3,std::bind(g,_3),_3,4,5);
    f2(10,11,12); //进行到f(12,g(12),12,4,5)的调用
    //绑定指向成员函数指针
    Foo foo;
    auto f3 = std::bind(&FOO::print_sum,&foo,95,_1);
    f3(5);
    //绑定指向数据成员指针
    auto f4 = std::bind(&Foo::data,-1);
    std::cout<<f4(foo)<<std::endl;
    //智能指针亦能用于调用被引用对象的成员
    std::cout<<f4(std::make_std<Foo>(foo))<<std::endl;
}
```

ambda表达式

lambda表达式可以说是c++11引用的最重要的特性之一，它定义了一个匿名函数，可以捕获一定范围的变量在函数内部使用，一般有如下语法形式：

```c++
auto func = [capture](params)opt->ret{cunc_body;};
```

其中func是可以当作lambda表达式的名字，作为一个函数使用，capture是捕获列表，params是参数表，opt是函数选项(mutable之类)， ret是返回值类型，func_body是函数体。

```c++
auto func1 = [](int a)->int{return a+1;};
auto func32 = [](int a){return a+2;};
cout<<func1(1)<<" "<<func2(2)<<endl;
```

如上代码，很多时候lambda表达式返回值是很明显的，c++11允许省略表达式的返回值定义。

lambda表达式允许捕获一定范围内的变量：

- []不捕获任何变量
- [&]引用捕获，捕获外部作用域所有变量，在函数体内当作引用使用
- [=]值捕获，捕获外部作用域所有变量，在函数内内有个副本使用
- [=, &a]值捕获外部作用域所有变量，按引用捕获a变量
- [a]只值捕获a变量，不捕获其它变量
- [this]捕获当前类中的this指针

```c++
int a = 0;
auto f1=[=](){return a;};//值捕获a
cout<<f1()<<endl;

auto f2 = [=](){return a++;}; //修改按值捕获的外部变量，error
auto f3 = [=]()mutable{return a++;};
```

代码中的f2是编译不过的，因为我们修改了按值捕获的外部变量，其实lambda表达式就相当于是一个仿函数，仿函数是一个有operator()成员函数的类对象，这个operator()默认是const的，所以不能修改成员变量，而加了mutable，就是去掉const属性。

还可以使用lambda表达式自定义stl的规则，例如自定义sort排序规则

```c++
struct A{
  	int a;
    int b;
};
int main(){
    vector<A>vec;
    std::sort(vec.begin(),vec.end(),[](const A&left,const A&right){
        return left.a<right.a;
    });
}
```

总结
std::function和std::bind使得我们平时编程过程中封装函数更加的方便，而lambda表达式将这种方便发挥到了极致，可以在需要的时间就地定义匿名函数，不再需要定义类或者函数等，在自定义STL规则时候也非常方便，让代码更简洁，更灵活，提高开发效率。

- 对C++11的mutex和RAII lock有过了解吗？

c++11关于并发引入了好多好东西，这里按照如下顺序介绍：

- std::thread相关
- std::mutex相关
- std::lock相关
- std::atomic相关
- std::call_once相关
- volatile相关
- std::condition_variable相关
- std::future相关
- async相关

std::thread

```c++
#include <iostream>
#include <thread>

using namespace std;

int main(){
	auto func = [](){
      for(int i=0;i<10;++i){
          cout<<i<<" ";
      }  
        cout<<endl;
    };
    std::thread t(func);
    if(t.joinable()){
        t.detach();
    }
    auto func1=[](int k){
        for(int i=0;i<k;++i){
            cout<<i<<" ";
        }
        cout<<endl;
    }
    std::thread tt(func1,20);
    if(tt.joinable()){	//检查线程可否被Join
		tt.join;
    }
    return 0;
}
```

上述代码中，函数func和func1运行在线程对象t和tt中，从刚创建对象开始就会新建一个线程用于执行函数，调用join函数将会阻塞主线程，直到线程函数执行结束，线程函数的返回值将会被忽略。如果不希望线程被阻塞执行，可以调用线程对象的detach函数，表示将线程和线程对象分离。

如果没有调用join或者detach函数，假如线程函数执行时间较长，此时线程对象的生命周期结束调用析构函数清理资源，这时可能会发生错误，这里有两种解决办法，一个是调用join()，保证线程函数的生命周期和线程对象的生命周期相同，另一个是调用detach()，将线程和线程对象分离，这里需要注意，如果线程已经和对象分离，那我们就再也无法控制线程什么时候结束了，不能再通过join来等待线程执行完。

这里可以对thread进行封装，避免没有调用join或者detach可导致程序出错的情况出现：

```c++
class ThreadGuard{
  	public:
    	enum class DesAction{join,detach};
    	ThreadGuard(std::thread&&t,DesAction a):t_(std::move(t)),action_(a){};
    	~ThreadGuard(){
            if(t_.joinable()){
                if(action_==DesAction::join){
                    t_.join();
                }else {
                    t_.detach;
                }
            }
        }
    	ThreadGuard(ThreadGuard&&)=default;
    	ThreadGuard& operator=(ThreadGuard&&)=default;
    	std::thread&get(){return t_;}
    	private:
    		std::thread t_;
    		DesAction action_;
};

int main(){
    ThreadGuard t(std::thread([]({
     	for(int i=0;i<10;++i){
            std::cout<<"thread guard"<<i<<" ";
        }   
        std::cout<<std::endl;
    }),ThreadGuard::DesAction::join));
}
```

c++11还提供了获取线程id，或者系统cpu个数，获取thread native_handle，使得线程休眠等功能

```c++
std::thread t(func);
cout<<"当前线程ID "<<t.get_id()<<endl;
cout<<"当前cpu个数"<<std::thread::hardware_concurrency()<<endl;
auto handle = t.native_handle(); //handle可用于pthread相关操作
std::this_thread::sleep_for(std::chrono::seconds(1));
```

std::mutex相关

std::mutex是一种线程同步的手段，用于保存多线程同时操作的共享数据。

mutex分为四种：

- std::mutex：独占的互斥量，不能递归使用，不带超时功能
- std::recursive_mutex：递归互斥量，可重入，不带超时功能
- std::timed_mutex：带超时的互斥量，不能递归
- std::recursive_timed_mutex：带超时的互斥量，可以递归使用

拿一个std::mutex和std::timed_mutex举例吧，别的都是类似的使用方式：

std::mutex:

```c++
#include <iostream>
#include <mutex>
#include <thread>

using namespace std;
std::mutex mutex_;

int main(){
    auto func1 = [](int k){
		mutex._lock();
        for(int i=0;i<10;++i){
            cout<<i<<" ";
        }
        cout<<endl;
        mutex._unlock();
    };
    std::thread threads[5];
    for(int i=0;i<5;++i){
		threads[i]=std::thread(func1,200);
    }
    for(auto&th:threads){
        th.join();
    }
    return 0;
}
```

std::timed_mutex:

```c++
#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>

using namespace std;
std::timed_mutex timed_mutex_;

int main(){
    auto func1 = [](int k){
		timed_mutex_.try_lock_for(std::chrono::milliseconds(200));
        for(int i=0;i<k;++i){
			cout<<i<<" ";
        }
        cout<<endl;
        timed_mutex_.unlock();
    };
    std:：thread threads[5];
    for(int i=0;i<5;++i){
        threads[i] = std::thread(func1,200);
    }
    for(auto&th:threads){
        th.join();
    }
    return 0;
}
```

std::lock

这里主要介绍两种RAII方式的锁封装，可以动态的释放锁资源，防止线程由于编码失误导致一直持有锁。

c++11主要有std::lock_guard和std::unique_lock两种方式，使用方式都类似，如下

```c++
#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>

using namespace std;
std::mutex mutex_;

int main(){
    auto func1 = [](int k){
        //std::lock_guard<std::mutex>lock(mutex_);
      	std::unique_lock<std::mutex>lock(mutex_) ;
        for(int i=0;i<k;++i){
			cout<<i<<" ";
        }
        cout<<endl;
    };
    std::thread threads[5];
    for(int i=0;i<5;++i){
        threads[i] = std::thread(func1,200);
    }
    for(auto&th:threads){
        th.join();
    }
    return 0;
}
```

std::lock_gurad相比于std::unique_lock更加轻量级，少了一些成员函数，std::unique_lock类有unlock函数，可以手动释放锁，所以条件变量都配合std::unique_lock使用，而不是std::lock_guard，因为条件变量在wait时需要有手动释放锁的能力，具体关于条件变量后面会讲到。

std::atomic相关

c++11提供了原子类型std::atomic<T>，理论上这个T可以是任意类型，但是我平时只存放整形，别的还真的没用过，整形有这种原子变量已经足够方便，就不需要使用std::mutex来保护该变量啦。看一个计数器的代码：

```c++
struct OriginCounter{	//普通计数器
    int count;
    std::mutex mutex_;
    void add(){
        std::lock_guard<std::mutex>lock(mutex);
        ++count;
    }
    void sub(){
        std::lock_guard<std::mutex>lock(mutex_);
        --count;
    }
    int get(){
        std::lock_guard<std::mutex>lock(mutex_);
        return count;
    }
};

struct NewCounter{	//使用原子变量的计数器
    std::atomic<int>count;
    void add(){
        ++count;
        //count.store(++count);这种也可以
    }
    void sub(){
        --count;
        //count.store(--count);
    }
    int get(){
		return count.load();
    }
};
```

是不是使用原子变量更加方便了呢？

std::call_once 相关

c++11提供了std::call_once来保证某一函数在多线程环境中只调用一次，它需要配合std::once_flag使用，直接看使用代码吧：

```c++
std::once_flag onceflag;

void CallOnce(){
    std::call_once(onceflag,[](){
       cout<<"call once"<<endl; 
    });
}

int main(){
    std::thread threads[5];
    for(int i=0;i<5;++i){
        threads[i] = std::thread(CallOnce);
    }
    for(auto&th:threads){
       th.join();
    }
     return 0;
}
```

volatile相关

貌似把volatile放在并发里介绍不太合适，但是貌似很多人都会把volatile和多线程联系在一起，那就一起介绍下吧。

volatile通常用来建立内存屏障，volatile修饰的变量，编译器对访问该变量的代码通常不再进行优化，看下面代码：

```c++
int *p = xxx;
int a = *p;
int b = *p;
```

a和b都等于p指向的值，一般编译器会对此做优化，把*p的值放入寄存器，就是传说中的工作内存(不是主内存)，之后a和b都等于寄存器的值，但是如果中间p地址的值改变，内存上的值改变啦，但a,b还是从寄存器中取的值(不一定，看编译器优化结果)，这就不符合需求，所以在此对p加volatile修饰可以避免进行此类优化。

**注意：**volatile不能解决多线程安全问题，针对特种内存才需要使用volatile，它和atomic的特点如下：

• std::atomic用于多线程访问的数据，且不用互斥量，用于并发编程中

• volatile用于读写操作不可以被优化掉的内存，用于特种内存中

std::condition_variable

条件变量是c++11引入的一种同步机制，它可以阻塞一个线程或者个线程，直到有线程通知或者超时才会唤醒正在阻塞的线程，条件变量需要和锁配合使用，这里的锁就是上面介绍的std::unique_lock。

这里使用条件变量实现一个CountDownLatch：

```c++
class CountDownLatch{
    public:
    	explicit CountDownLatch(uint32_t count):count_(count){}
    	
    	void CountDown(){
            std::unique_lock<std::mutex>lock(mutex_);
            --count_;
            if(count_==0){
                cv.notify_all();
            }
        }
    	void Await(unit32_t time_ms=0){
			std::unique_lock<std::mutex>lock(mutex_);
            while(count_>0){
                if(time_ms>0){
                    cv.wait_for(lock,std::chrono::milliseconds(time_ms));
                }else{
                    cv.wait(lock);
                }
            }
        }
    
    	uint32_t GetCount()const{
            std::unique_lock<std::mutex>lock(mutex_);
        	return count_;
        }
    private:
    	std::condition_variable cv_;
    	mutable std::mutex mutex_;
    	uint32_t count_=0;
};
```

关于条件变量其实还涉及到通知丢失和虚假唤醒问题，因为不是本文的主题，这里暂不介绍，大家有需要可以留言。

std::future相关

c++11关于异步操作提供了future相关的类，主要有std::future、std::promise和std::packaged_task，std::future比std::thread高级些，std::future作为异步结果的传输通道，通过get()可以很方便的获取线程函数的返回值，std::promise用来包装一个值，将数据和future绑定起来，而std::packaged_task则用来包装一个调用对象，将函数和future绑定起来，方便异步调用。而std::future是不可以复制的，如果需要复制放到容器中可以使用std::shared_future。

**std::promise与std::future配合使用**

```c++
#include <funtional>
#include <future>
#include <iostream>
#include <thread>

using namespace std;

void func(std::future<int>&fut){
    int x = fut.get();
    cout<<"value:"<<x<<endl;
}
int main(){
    std::promise<int>prom;
    std::future<int>fut = prom.get_future();
    std::thread t(func,std::ret(fut));
    prom.set_value(144);
    t.join();
    return 0;
}
```

std::packaged_task与std::future配合使用

```c++
#include <functional>
#include <future>
#include <iostream>
#include <thread>

using namespace std;

int func(int in){
    return in+1;
}

int main(){
    std::packaged_task<int(int)>task(func);
    std::future<int>fut = task.get_futre();
    std::thread(std::move(task),5).detach();
    cout<<"result"<<fut.get()<<endl;
    return 0;
}
```

更多关于future的使用可以看我之前写的关于线程池和定时器的文章。

**三者之间的关系**

std::future用于访问异步操作的结果，而std::promise和std::packaged_task在future高一层，它们内部都有一个future，promise包装的是一个值，packaged_task包装的是一个函数，当需要获取线程中的某个值，可以使用std::promise，当需要获取线程函数返回值，可以使用std::packaged_task。

async相关

async是比future，packaged_task，promise更高级的东西，它是基于任务的异步操作，通过async可以直接创建异步的任务，返回的结果会保存在future中，不需要像packaged_task和promise那么麻烦，关于线程操作应该优先使用async，看一段使用代码：

```c++
#include <functional>
#include <future>
#include <iostream>
#include <thread>

using namespace std;

int func(int in){return in +1;}

int main(){
    auto res = std::async(func,5);
    //res.wait();
    cout<<res.get()<<endl; //阻塞直到函数返回
    return 0;
}
```

使用async异步执行函数是不是方便多啦。

async具体语法如下：

```c+++
async(std::launch::async|std::launch::deferred,func,args...);
```

第一个参数是创建策略：

- std::launch::async表示任务执行在另一线程
- std::launch::deferred表示延迟执行任务，调用get或者wait时才会执行，不会创建线程，惰性执行在当前线程。

如果不明确指定创建策略，以上两个都不是async的默认策略，而是未定义，它是一个基于任务的程序设计，内部有一个调度器(线程池)，会根据实际情况决定采用哪种策略。

若从 std::async 获得的 std::future 未被移动或绑定到引用，则在完整表达式结尾， std::future的析构函数将阻塞直至异步计算完成，实际上相当于同步操作：

```c++
std::async(std::launch::async,[]{f();});  //临时量的析构函数等待f()
std::async(std::launch::async,[]{g();});   //f()完成前不开始
```

**注意：**关于async启动策略这里网上和各种书籍介绍的五花八门，这里会以cppreference为主。

• 有时候我们如果想真正执行异步操作可以对async进行封装，强制使用std::launch::async策略来调用async

 std::thread使线程的创建变得非常简单，还可以获取线程id等信息。

**•** std::mutex通过多种方式保证了线程安全，互斥量可以独占，也可以重入，还可以设置互斥量的超时时间，避免一直阻塞等锁。

**•** std::lock通过RAII技术方便了加锁和解锁调用，有std::lock_guard和std::unique_lock。

**•** std::atomic提供了原子变量，更方便实现实现保护，不需要使用互斥量

**•** std::call_once保证函数在多线程环境下只调用一次，可用于实现单例。

**•** volatile常用于读写操作不可以被优化掉的内存中。

**•** std::condition_variable提供等待的同步机制，可阻塞一个或多个线程，等待其它线程通知后唤醒。

**•** std::future用于异步调用的包装和返回值。

**•** async更方便的实现了异步调用，异步调用优先使用async取代创建线程。

- 对C++11的智能指针了解多少，可以自己实现一个智能指针吗？

智能指针是C++中一项很常用的技术，合理的使用智能指针可以更方便的管理内存，降低内存泄漏的风险。

01 智能指针的种类

unique_ptr

shared_ptr

weak_ptr

shared_ptr内部是使用引用计数来记录托管指针被引用的次数，当托管指针的引用计数为0时会释放托管的内存，这里通过gcc源码探究shared_ptr内部究竟是如何实现的内存引用计数功能。

总结：

​		shared_ptr内部使用 __shared_count中的 _Sp_counted_base对象来控制托管指针, _Sp_counted_base内部有 _M_use_count和 _M_weak_count, _M_use_count表示托管指针的引用计数， _M_weak_count表示  _Sp_counted_base的引用计数，

_M_use_count为0的时候释放托管指针内存指向的内存， _M_weak_count为0时释放

_Sp_counted_base指向的内存，这里 _Sp_counted_base的生命线一般不会短与 

shared_ptr的生命线。

注意：文中所说的释放指针指向的内存不太准确，这里表示delete *ptr，既调用析构函数又释放相应内存。

- enum 和 enum class有什么区别？

1. 什么是枚举类型? 
答：如果**一个变量只有几种可能的值**，那么就可以定义为枚举类型，比如：性别只有男和女，那么就可以将性别定义为一种枚举类型，其中男和女就是性别所包含的变量。所谓”枚举”是指将变量的值一一列举出来，变量的值只能在列举出来的值的范围内。在C++中，枚举类型分为不限定作用域（enum）和限定作用域（enum class）
2. enum与enum class的区别? （为什么需要限定作用域？） 
答：枚举作用域是指枚举类型成员名字的作用域，起自其声明之处，终止枚举定义结束之处。enum与class enum区别在于是否限定其作用域。C语言规定，枚举类型（enum）的成员的可见范围被提升至该枚举类型所在的作用域内。这被认为有可能污染了外部的作用域，为此，C++11引入了枚举类(enum class)解决此问题。


enum

```c++
enum Color{
  	RED=1,
    GREEN=2,
    BLUE=4
};
Color clr = GREEN;	//只能拿枚举常量给枚举变量赋值，才不会出现类型的差异
clr = 5;//error

enum Sex{
    GIRL,
    BOY
};
//报错，重定义
enum Student{
  	GIRL,
    BOY
};
```

编译错误的原因在于Sex与Student都处在同一作用域下，成员变量重定义。

这便体现C++11引入枚举类(enum class)的重要性，enum class能够有效对枚举作用域进行限定，避免了枚举成员的重定义。

枚举中每个成员(标识符)结束符是“,”而不是”;” 最后一个成员可省略”,”
初始化时可以赋负数，以后的标识符仍依次加1。
枚举变量只能取枚举说明结构中的某个标识符常量。
在外部可以对枚举变量进行赋值，但需要进行类型转换。

Student x = GIRL;

x = (Student)4;

枚举常数可以隐式转换为int，但是int不可以隐式转换为枚举值。
为枚举中的每个名称分配一个整数值，该值与其在枚举中的顺序相对应。默认情况下，第一个值分配0，下一个值分配1，依次类推，但也可以显示设置枚举名称的值。
枚举值可以用来作判断比较。



enum class

```c++
enum class Color{
    RED=1,
    GREEN=2,
    BULE=4
};
Color clr = Color::GREEN;//enum classs 必须加上作用域限定符
//clr = 4;error
enum class Sex{
    GIRL,
    BOY
};
enum class Student{
    GIRL,
    BOY
};
int main(int argc,char*argv[]){
    Sex a = Sex::GRIL;
    Student b = Student::GRIL;
}
```

是否可以隐式转换为Int类型

enum方式声明的枚举类型可以隐式的转换为int类型

```c++
int a = GREEN;
int b = clr;
```

枚举定义将被限制在枚举作用域内，并且不能隐式转换为整数类型，但是可以显式转化为整数类型，

```
int a = Color::GREEN; //编译报错
int b = clr; //编译报错
int b = int(clr);//正确，显示将enum class转换为整数
```

22.STL

- C++直接使用数组好还是使用std::array好？std::array是怎么实现的？

```c++
const int MAX = 10;
const int MAX2 = 9;
void fuck(const std::array<int,MAX>&arr)
{
    for(int a::arr){
		shit(a);
        ....
    }
    int bitch  =std::accumulae(arr.begin(),arr.end(),0,crap);
}
int main()
{
    std::array<int,MAX>arr1;
    std::array<int,MAX2>arr2;
    int *ptr;
    fuck(arr1);//ok
	fuck(arr2);//won't compile
    fuck(ptr);//won't compile
}
```

std::array：好看，好维护，安全,提供了各种迭代器、算法、操作方法等。不会隐式转成指针。

std::array是一个模板类，封装固定大小数组的容器。

与编译器优化，处理器体系结构紧密联系。例如，内存分配与高速缓存或虚拟内存也对齐的程度如何，以避免额外的工作。

- std::vector最大的特点是什么？它的内部是怎么实现的？resize和reserve的区别是什么？clear是怎么实现的？

  vector能实现动态增长，当插入新元素的时候，如果空间不足，那么vector会重新申请更大的一块内存空间，将原空间的数据拷贝到新空间，是否旧空间的数据，再把新元素插入新申请空间。

  resize():改变的是size()与capacity()的大小

  1）比原来的变小，后面的会被截断

  2）比原来的变大，后面的会被填充新的东西

  reserve()：改变的只是capacity()的大小，size()和内容不会改变

  vector::erase()用于清空容器中的内容以及释放内存,并返回指向删除元素的下一个元素的迭代器。

  vector::clear()函数的作用是清空容器中的内容，但如果是指针对象的话，并不能清空其内容

  clear：

  对于STL中的vector调用clear时，内部是如何操作的？若想将其内存释放，该如何操作？

  由于对象的指针不是内建对象，所以进行遍历析构。

  for ( ; first <last; ++first)    //遍历元素进行析构

  destroy(&*first);                  //!!!!!关键句!!!!!!!!!

  *iterator是元素类型，&*iterator是元素地址，也就是一个指针。之后调用&*iterator->~T();所以可知当vector中所存储的元素为对象的时候，调用clear（）操作的时候系统会自动调用析构函数。但是当存储元素是指针的时候，指针指向的对象就没法析构了。因此需要释放指针所指对象的话，需要在clear操作之前调用delete。

- deque的底层数据结构是什么？它的内部是怎么实现的？

  双端数组，可以对头端进行插入删除操作

  deque内部有个**中控器**，维护每段缓冲区中的内容，缓冲区中存放真实数据

  中控器维护的是每个缓冲区的地址，使得使用deque时像一片连续的内存空间

  

- map和unordered_map有什么区别？分别在什么场景下使用？

  map:map内部实现了一个红黑树（红黑树是非严格平衡二叉搜索树，而AVL是严格平衡二叉3搜索树），红黑树具有自动排序的功能，因此map内部的所有元素都是有序的，红黑树的每一个节点都代表着map的一个元素。map中的元素是按照[二叉搜索树](https://baike.baidu.com/item/二叉搜索树/7077855?fr=aladdin)（又名二叉查找树、二叉排序树，特点就是左子树上所有节点的键值都小于根节点的键值，右子树所有节点的键值都大于根节点的键值）存储的，使用中序遍历可将键值按照从小到大遍历出来。

  unordered_map: unordered_map内部实现了一个哈希表（也叫散列表，通过把关键码值映射到Hash表中一个位置来访问记录，查找的时间复杂度可达到O(1)，其在海量数据处理中有着广泛应用）。因此，其元素的排列顺序是无序的。

  优缺点以及适用处
  map：

  优点：

  有序性，这是map结构最大的优点，其元素的有序性在很多应用中都会简化很多的操作
  红黑树，内部实现一个红黑书使得map的很多操作在lgn的时间复杂度下就可以实现，因此效率非常的高
  缺点： 空间占用率高，因为map内部实现了红黑树，虽然提高了运行效率，但是因为每一个节点都需要额外保存父节点、孩子节点和红/黑性质，使得每一个节点都占用大量的空间

  适用处：对于那些有顺序要求的问题，用map会更高效一些unordered_map：

  优点： 因为内部实现了哈希表，因此其查找速度非常的快
  缺点： 哈希表的建立比较耗费时间
  适用处：对于查找问题，unordered_map会更加高效一些，因此遇到查找问题，常会考虑一下用unordered_map

  内存占有率的问题就转化成红黑树 VS hash表 , 还是unorder_map占用的内存要高。
  但是unordered_map执行效率要比map高很多
  对于unordered_map或unordered_set容器，其遍历顺序与创建该容器时输入的顺序不一定相同，因为遍历是按照哈希表从前往后依次遍历的

  

- list的使用场景？std::find可以传入list对应的迭代器吗？

- 采用动态存储分配，不会造成内存浪费和溢出

- 链表执行插入和删除操作十分方便，修改指针即可，不需要移动大量元素

- 链表灵活，但是空间(指针域) 和 时间（遍历）额外耗费较大

  ST双向链表，不支持随机访问，在任何位置插入和删除效率都比较好。可以传入。

- string的常用函数

  

  ```c++
  //赋值
  string str1;
  str1 = "hello world";
  string str4;
  str4.assign(str1);
  //字符串拼接（append)
  string str1 = "wo";
  string str3 = "i";
  str3.append(str1);
  //查找
  int find(const string&str,int pos=0)const;
  string str = "adcdefgde";
  int pos = str1.find("de");
  //替换
  string&replace(int pos,int n,const string &str);
  
  string str1  = "abcdse";
  
  str1.replace(1,3,"1111");
  
  
  
  //字符串比较 (int compare(const string&s)const;)
  string s1 = "hello";
  string s2 = "alllo";
  int ret = s1.compare(s2);
  //字符串存取(char&operator[](int n), char&at(int n))
  string str = "hello world";
  str.at(0) = 'x';
  str[0] = 'x';
  //字符串插入 (string insert(int pos,const string&str));
  string str = "hello";
  str.insert(1,"1111");
  //字符串删除 (string &erase(int pos,int n=npos);)
  str.erase(1,3);
  //子串 (string substr(int pos=0,int n=npos)const;)
  string str = "abcdefg";
  string substr = str.substr(1,3);
  ```

  



