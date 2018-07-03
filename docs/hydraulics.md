Oil
====
Coastal Economy AW-32
http://petroleumservicecompany.com/content/pdfs/PDS-COASTAL-ECONOMY-AW-HYDRAULIC-OILS.pdf

Reservoir
====
Buyers SMR25S (18 G?)
https://www.buyersproducts.com/product/side-mount-steel-reservoir-3090

Filters
====
Zinga SE-10
http://www.womackmachine.com/products/hydraulics/filters-and-breathers/spin-on-filters/zinga-spin-on-filter-elements/
2 Beta (5 um)
75 Beta (19 um)

Valves
====
Hydraforce sp10-47c-8t-n-12ds
http://www.hydraforce.com/proport/Prop-pdf/2-112-1.pdf

- 3600 psi operating pressure
- 6 gpm max flow
- initial flow starts at 0.4 A at 12 V
--  max flow at 1.1 to 1.2 A at 12 V
- specs quoted with 100 Hz dither

Pumps
====
Prince SPD263A9H1L
http://www.surpluscenter.com/Hydraulics/Hydraulic-Pumps/Multi-Section-Pumps/3-869-cu-in-PRINCE-HYD-FRONT-PUMP-SPD263A9H1L-9-5013-C.axd

- 3.869 cu in displ
- 33.5 GPM at 2000 RPM
- 2500 PSI
- 3000 RPM
- IN SAE 24 side
- OUT SAE 16 side
- SAE B 2 bolt mount
- CCW rotation
- 7/8" -13 tooth splined shaft 1.63" long
- 7.21" x 6.87" x 5.71"
- Shpg 12 lbs

Prince SP20B23A9H9L
http://www.surpluscenter.com/Hydraulics/Hydraulic-Pumps/Gear-Pumps/1-403-cu-in-PRINCE-SP20B23A9H9L-HYD-PUMP-9-1900-C.axd

- 1.403 cu in displ
- 11.60 GPM at 2000 RPM
- 2500 PSI
- 3500 RPM max
- IN SAE 16 side
- OUT SAE 12 side
- SAE A 2 bolt mount
- CCW rotation
- 5/8" 9 tooth x 1-1/4" splined shaft
- 5"x5-1/8"x4-5/8"
- Shpg 9 lbs


Cylinders
====

Knee: DBH-2512-WT
http://www.daltonhydraulic.com/dalton-welded-tube-cylinder-2-5-bore-x-12-stroke
2.5" bore, 1.5" rod, 12" travel, 20" retracted length

Thigh: DBH-3514-WT
http://www.daltonhydraulic.com/dalton-welded-tube-cylinder-3-5-bore-x-14-stroke
3.5" bore, 1.75" rod, 14" travel, 24" retracted length

Hip: DBH-2008-WT
http://www.daltonhydraulic.com/dalton-welded-tube-cylinder-2-bore-x-8-stroke
2" bore, 1.25" rod, 8" travel, 16" retracted length


Load calculations
====
Assuming 2500 psi, 2000 rpm, combined flow is 45 GPM and using this table:

http://www.womackmachine.com/engineering-toolbox/data-sheets/electric-motor-size-for-hydraulic-pump-drive/

gives 78 HP.

Starting with the valves, max flow is 6 GPM x 18 joints, giving 108 GPM, way more than the pumps can handle.

Similar number (83) from here:

http://www.hydraulicspneumatics.com/hydraulic-pumps-amp-motors/sizing-motors-hpus

combined pump displacement: 
5.272 = 3.869 + 1.403

required torque (assuming 80% efficiency):
~218 = (2500 * 5.272) / (6.28  * 12 * 0.8) [lb ft]

hp at 2000 rpm
~83 = (218 * 2000) / 5250. [hp]

http://www.hydraulicspneumatics.com/200/TechZone/ManifoldsHICs/Article/False/6405/TechZone-ManifoldsHICs

says oversize the combustion engine by 2.5x the electric, so assuming 83 hp, we should be looking at 207 HP.

Getting an estimated flow for a gait seems tricky without knowing how hard the valves/pumps work with loaded legs. Assuming 100% duty cycle of just the hips,3 swinging back, 3 swinging forward at full speed, we're already looking at 36 GPM (80% of the flow going to the hips).
