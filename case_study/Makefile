LATEX = latex

DVIPS = dvips

PDFFLAGS = -dCompatibilityLevel=1.4 -dPDFSETTINGS=/prepress \
           -dCompressPages=true -dUseFlateCompression=true  \
           -dEmbedAllFonts=true -dSubsetFonts=true -dMaxSubsetPct=100

%.dvi: %.tex
	$(LATEX) $<

%.ps: %.dvi
	$(DVIPS) -o $@ $<

%.pdf: %.ps
	ps2pdf $(PDFFLAGS) $<

all:	book.tex
	latex book
	makeindex book
	latex book
	dvips -t letter -Ppdf -o complexity.ps book
	#evince complexity.ps
	open complexity.ps

pdf:
	ps2pdf $(PDFFLAGS) complexity.ps

hevea:
	rm -rf html
	mkdir html
	hevea -e latexonly htmlonly book
	imagen -png book
	hacha book.html
	cp up.png next.png back.png html
	mv index.html book*.html book*.png *motif.gif html

DEST = /home/downey/public_html/greent/complexity

distrib:
	rm -rf dist
	mkdir dist dist/tex dist/tex/figs
	rsync -a complexity.pdf html dist
	rsync -a Makefile book.tex latexonly htmlonly dist/tex
	rsync -a figs/*.fig figs/*.eps dist/tex/figs
	cd dist; zip -r complexity.tex.zip tex; rm -r tex
	rsync -a dist/* $(DEST)
	chmod -R o+r $(DEST)/*

clean:
	rm -f *~ *.aux *.log *.dvi *.idx *.ilg *.ind *.toc

