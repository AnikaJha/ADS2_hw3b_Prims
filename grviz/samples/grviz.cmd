@SET PATH=%PATH%;c:\programs\Graphviz2.38\bin


@rem dot -Tps %1 -o %1.ps
dot -Tgif %1 -o %1.gif