
file="manual.tex"

pdflatex -interaction=nonstopmode $file
pdflatex -interaction=nonstopmode $file

mv manual.pdf ../../
