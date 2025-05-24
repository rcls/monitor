
KICAD_PRO:=$(wildcard *.kicad_pro)
PROJECTS=$(KICAD_PRO:%.kicad_pro=%)

$(PROJECTS:%=%.all): %.all: out/%_bom.csv out/%_cpl.csv out/%_gerber.zip

.PHONY: $(PROJECTS:%=%.all) $(PROJECTS:%=%.drc)

$(PROJECTS:%=%.drc): %.drc: %.rpt
	! grep 'Found [^0]' $<

out/%.rpt: %.kicad_pcb
	mkdir -p out
	kicad-cli pcb drc $<

#%_bom.csv: %_bom_raw.csv filter_bom.py
#	./filter_bom.py $< $@

$(PROJECTS:%=out/%_bom.csv): out/%_bom.csv: %.kicad_sch *.kicad_sch Parts.kicad_sym
	mkdir -p out
#	kicad-cli sch export bom --exclude-dnp --labels Comment,Designator,Footprint,'JLCPcb Part' --ref-range-delimiter='' -o $@ $<
	kicad-cli sch export bom --preset JLCPCB --exclude-dnp --labels Comment,Designator,Footprint,'JLCPcb Part' --ref-range-delimiter='' -o $@ $<

out/%_gerber.zip: %.kicad_pcb
	mkdir -p out
	mkdir temp/
	kicad-cli pcb export gerbers --use-drill-file-origin --board-plot-params -o temp $<
	kicad-cli pcb export drill -o temp/ $<
	zip -j $@ temp/*
	rm -r temp

out/%_cpl_all.csv: %.kicad_pcb
	mkdir -p out
	kicad-cli pcb export pos --units mm --format csv --exclude-dnp --side both -o $@ $<

out/%_cpl.csv: out/%_cpl_all.csv out/%_bom.csv filter_pos.py
	mkdir -p out
	./filter_pos.py $< out/$*_bom.csv $@

.PHONY: clean
clean:
	rm -rf out temp
