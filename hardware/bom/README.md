# Preliminary Bill of Materials

This directory contains [`motor-bom-source.xlsx`](motor-bom-source.xlsx), the unmodified Chinese workbook supplied with the first 4310 mechanical package. The explanation and summary below provide an English entry point without creating a second BOM that could drift away from the source.

The source has no column headers. Based on its totals, column D appears to be the estimated **line cost in CNY**, not a unit price. Purchased mechanical line costs total CNY 114.485, which the source rounds to CNY 115. The source separately estimates PCB assembly at CNY 6 and electronics components at CNY 80, for an electronics subtotal of CNY 86 and a stated overall target of no more than CNY 250.

These figures are preliminary prototype estimates. Supplier, manufacturer part number, material, machining process, currency date, tax, shipping, and replacement status are mostly absent. Quantities and costs for printed/machined enclosure parts are grouped rather than allocated by part. Do not use this BOM as a purchasing or manufacturing release until those fields are completed and checked against the mechanical revision.

## Cost Summary

| Category | Estimate (CNY) | Source basis |
| --- | ---: | --- |
| Mechanical parts | 115 | Rounded source subtotal |
| PCB assembly | 6 | Source estimate; note says five-board minimum costs 33 |
| Electronics components | 80 | Source row labelled BOM |
| Electronics subtotal | 86 | Source subtotal |
| Overall target | ≤250 | Source target; exceeds listed subtotals, likely allowing manufacturing/contingency |

Future releases should separate electrical and mechanical BOMs and add supplier, manufacturer, MPN, material, process, finish, revision, unit price, extended price, and verified-alternative fields.
