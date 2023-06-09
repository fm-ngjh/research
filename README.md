# 概要
研究で作成しているシステムです．有限要素法に対応したPhysX 5のライブラリを用いてVR物体との力学的なインタラクションが可能な変形するVR手モデルの実装を目指しています．

2023年4月より開発を始めました．現在は剛体と変形物体が干渉する際の諸問題について調査し，2つの変形物体で剛体を挟んで持ち上げるといったシミュレーションが問題なく実現できる状況を模索しています．

本システムはPhysX 5のサンプルコードであるSnippet*.cppおよびSnippet*.hをもとに開発している部分が多く，sourceフォルダ内のcppファイルの全部分を私が書いているわけではないことに注意してください．/research/PX_FLOAT_POINT_PRECISE_MATH_True/common/snippets/内に一連のサンプルコードがあります．本システムのmain.cppはSnippetSoftbody.cppとSnippetHelloWorld.cppを元にして開発を始めました．

主に/research/tree/main/PX_FLOAT_POINT_PRECISE_MATH_True/main/source/内のmain.cppとrender.cppを書いています．

ネット上のPhysX 5開発における情報は乏しいため，サンプルコードと公式のドキュメントを参照しながら試行錯誤して実装しています．


# 実行結果
/research/PX_FLOAT_POINT_PRECISE_MATH_True/common/Debug/内に生成されるmain.exeファイルを実行した結果を示します．

剛体(白い立方体)と変形物体(青い立方体)を接続した物体で左右から把持対称となる剛体を挟み込み，持ち上げます．この際，把持対象物の揺れや摩擦力不足以外が原因と考えられる落下が発生するため，現在はその原因の分析や対処方法の模索を行っています．

![image](https://github.com/fm-ngjh/research/assets/135797163/63037669-5ad3-40f3-bdc4-e2b2ad99d880)
![image](https://github.com/fm-ngjh/research/assets/135797163/6a0c17b9-969a-4894-a159-254d83ccab5e)

