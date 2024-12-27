mapping: mapping.c
	gcc mapping.c -lm -lSDL2 -O3 -o mapping

clean:
	@rm -rf heat_map.raw
	@rm -rf mapping
