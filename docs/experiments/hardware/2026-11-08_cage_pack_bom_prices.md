# Cage pack BOM prices, 2026-11-08

How the cage pack purchase data in `hardware/cage_pack/bom_data.py` was priced.

These notes were moved out of source comments and docstrings in `hardware/` at commit
`23e253c1`. Names, line numbers and numbers are as they were in the code then.

## Notes

### `cage_pack/bom_data.py`

Docstring at line 1:

```text
Purchase data for the cage pack BOM: names, links and unit prices, checked 2026-11-08.

Prices are USD, quantity 1 unless the note says otherwise. Pack items (McMaster screws,
Amazon kits) carry the per-unit price so the BOM subtotal counts only what the build uses;
the note gives the pack size and pack price, which is what the order actually costs.
Owned parts have no price, so the BOM total is the cost to buy what is missing.

Sources: Digi-Key and McMaster product pages, 18650BatteryStore, Stereolabs store, Amazon
search results (first relevant listing; Amazon prices move daily). Mouser blocks automated
browsing and Mini-Box shows no OpenUPS price without a cart, so neither is priced here.
```

Line 233, `above PCBA: dict = {`:

```text
The custom PCBA as one line, priced from the last JLCPCB order rather than part by part:
the Mr Stabs Mk2 board (W2026100921070917, 2026-10-09) cost $452.79 delivered for 5 bare
PCBs with 2 assembled ($289.78 boards and assembly, $39.46 DHL, $105.43 duty, $18.12 tax),
about $226 per assembled board. That board had 30 BOM lines; this one has about 20, so
expect a little less. Re-quote at JLCPCB once the board is laid out.
```
