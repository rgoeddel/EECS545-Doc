NAME=finalReport
OPTS=-halt-on-error -shell-escape -interaction nonstopmode

all: $(NAME).pdf

$(NAME).pdf: $(NAME).tex literature.bib
	pdflatex $(OPTS) $(NAME).tex
	bibtex $(NAME)
	pdflatex $(OPTS) $(NAME).tex
	pdflatex $(OPTS) $(NAME).tex

clean:
	rm -rf *.aux *.bbl *.blg *.dvi *.log *.pdf *~

