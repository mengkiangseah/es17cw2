#!/bin/bash

alias ts="./typeset.sh"

pdflatex main.tex

if [ "$1" = "-f" ]
	then
		bibtex main
		pdflatex main.tex
		pdflatex main.tex
fi

mkdir latex-outputs

mv *.aux latex-outputs
mv *.blg latex-outputs
mv *.log latex-outputs
mv *.out latex-outputs