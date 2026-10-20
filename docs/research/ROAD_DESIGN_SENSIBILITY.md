# Road design sensibility reference (for reviewing generated OpenDRIVE networks)

This file gives an LLM reviewer of OdrForge output (OpenStreetMap converted to lane-level OpenDRIVE)
a sourced sense of what real road networks look like and how they are used. The test areas are
Munich (DE), London (GB, left-hand traffic), the Frankfurter Kreuz (DE motorway interchange), the
Stelvio pass (IT, mountain hairpins), Tokyo Shibuya (JP, left-hand traffic, elevated expressway) and
downtown Los Angeles (US, freeways and a street grid). Use it to decide whether something in a plan
view or in the XML is a **defect** or a **legitimate real-world feature**. Judge it from the points of
view of a car driver, a pedestrian, a wheelchair user, a cyclist and a driving simulator (CARLA, esmini).

**How to read it**
- Values are given as ranges (typical / minimum / maximum) with the region or standard they come from.
  Design values are minimums for *new* construction. Existing streets, and especially old towns, are
  often below them. A value below a design minimum is a reason to look closer, not proof of a defect.
- US customary values are converted (1 ft = 0.305 m, 1 mph = 1.609 km/h). Values marked **(sec.)** come
  from a secondary source that quotes the standard (a state DOT, council, presentation or summary).
  Values marked **(unverified)** could not be confirmed in any source and are only plausible.
- Many values depend on design speed. Where they do, the speed column says so.
- Source tags such as [S12] point to the numbered list at the end. Wording is paraphrased.

---

## 1. Lane widths

| Element | Value | Unit | Region / standard | Condition |
|---|---|---|---|---|
| Urban arterial lane | 3.0 - 3.6 (10 - 12 ft) | m | US AASHTO Green Book, quoted by FHWA [S3] | lower value acts as the minimum |
| Low-speed, low-volume rural lane | down to 2.7 (9 ft) | m | US AASHTO via FHWA [S3] | little or no truck traffic |
| Urban lane, recommended | 3.05 (10 ft) | m | US NACTO [S4] (sec.) | lanes > 3.35 m (11 ft) "should not be used" in cities |
| Urban truck / transit route | 3.35 (11 ft), one lane per direction | m | US NACTO [S4] (sec.) | designated truck or bus routes |
| Through lane next to a turn lane | 2.75 - 2.9 (9 - 9.5 ft) | m | US NACTO [S4] (sec.) | "select cases" |
| State highway / ramp lane | 3.66 (12 ft) min, 3.35 (11 ft) allowed | m | US Caltrans HDM 301.1 [S14] | 11 ft only <= 40 mph urban, < 250 trucks/lane |
| Motorway / trunk lane, standard | 3.65 | m | UK DMRB CD 127 [S17] | additional lanes > 100 m long use 3.65 m |
| Ghost-island through lane | 3.0 - 3.65 | m | UK CD 123 6.8 [S19] | not WS2+1; 3.5 on climbing lanes |
| Right-turn lane (ghost island) | 3.5 desirable; 3.0 new; 2.5 existing | m | UK CD 123 6.10-6.11 [S19] | urban max 5.0; rural > 85 km/h max 3.65 |
| Roundabout entry lane at give-way | 3.0 - 3.5 (multi-lane); 4.5 (single) | m | UK CD 116 3.14 [S21] | |
| Segregated left-turn lane (UK slip) | >= 3.5 | m | UK CD 116 6.15 [S21] | |
| Lane widening on curves | +0.3 per lane | m | UK CD 109 [S18] | standard width, 90 m < R < 150 m |
| Motorway lane (RQ 31) | 3.75 | m | DE RAA 2008 via Bundestag WD [S37] | 4-lane motorway; hard shoulder 3.00 |
| Motorway lane, range | 3.25 / 3.50 / 3.75 | m | DE RAA (sec., lecture slides) [S37] | inner lanes narrower than outer |
| Two-lane urban main road carriageway | 6.50 (2 x 3.25); 7.00 if bus/lorry meet often | m | DE RASt 06 + FGSV 2024 supplement [S35] | below 6.50 m two buses cannot pass |
| Car/car pinch point | 4.10 - 4.75 total | m | DE RASt 06 (sec.) | narrowing for traffic calming |
| Lane by road type | A, B 3.75; C 3.50 - 3.75; D 3.25; E ~3.00; F 2.75 - 3.25 | m | IT DM 5/11/2001 [S32] | A/B/C2/D read clearly; E/F cells garbled in the text extract |
| Lane by road class | Type 1 Class 1: 3.5 (3.75 special) | m | JP Road Structure Ordinance (sec.) [S28] | expressways |
| Lane by road class | Type 3 Cl.2 3.25; Cl.3 3.0; Cl.4 2.75; Type 4 Cl.1 3.25; Cl.2-3 3.0; small-car roads 2.75 | m | JP RSO as adopted by Osaka [S27] | Type 4 = urban streets |
| Turn / speed-change lane | 3.0 (2.5 on small-car roads) | m | JP RSO as adopted by Osaka [S27] | |
| Bus lane, curbside / offset | >= 3.35 (11 ft) / >= 3.05 (10 ft); 2.75 (9 ft) at stops | m | US NACTO [S7][S8] | median bus lane >= 3.35 per direction |
| Shared bus-bike lane | 3.05 - 3.35 offset, up to 3.66 curbside | m | US NACTO [S8] (sec.) | avoid 4.0 - 4.6 m (13 - 15 ft) |
| Bus lane | 3.0 abs. min; 3.25 normal min; 4.5 desirable | m | UK Edinburgh street design guidance [S85] | contra-flow lanes |
| Parking lane / bay depth | 2.00 parallel; 4.80 at 45 deg; 5.00 perpendicular | m | IT DM 2001 3.4.7 [S32] | aisle 3.50 (parallel) to 6.00 (perpendicular) |

Real-world usage. A 2007 TRB study of 408 miles of US urban and suburban arterials found no general
increase in crashes with lanes narrower than 3.6 m (12 ft) [S16]. Built urban lanes of 2.75 - 3.25 m are
normal in Europe and Japan. No field survey of measured lane widths was found (thin).

**Judge as defect if**
- A general-traffic lane is < 2.5 m over more than a short pinch point, or > 4.5 m with no reason
  (single-lane entry, bus stop, parking, turn shelter). Widths of 5 - 7 m usually mean two lanes merged
  into one, or a lane plus a parking or bus lane the converter missed.
- A motorway or expressway lane is < 3.25 m outside roadworks, or a ramp lane is < 3.5 m on a tight curve.
- Lane widths jump by more than about 0.5 m at a lane-section boundary with no taper (see section 2).

**Legitimate if**
- Old-town or residential streets have 2.5 - 3.0 m lanes, or a single 3.0 - 4.5 m lane without a centre line.
- Turn lanes are narrower than through lanes (UK 2.5 m existing, JP 2.5 - 3.0 m), or urban right-turn
  lanes are up to 5.0 m wide as a shelter for lorries (UK).
- Inner motorway lanes are 3.25 - 3.5 m and the outer lane is 3.75 m (DE).

---

## 2. Lane add, drop and merge tapers

| Element | Value | Unit | Region / standard | Condition |
|---|---|---|---|---|
| Lane-reduction taper | L = W*S^2/60 | ft (W ft, S mph) | US MUTCD 3B.09 [S2] | speed < 45 mph |
| Lane-reduction taper | L = W*S | ft | US MUTCD 3B.09 [S2] | speed >= 45 mph |
| Worked values, W = 12 ft (3.66 m) | 25 mph: 38 m; 30 mph: 55 m; 40 mph: 98 m; 45 mph: 165 m; 65 mph: 238 m | m | computed from [S2] | |
| Merging / shifting / shoulder taper | >= L / >= 0.5 L / >= 0.33 L | - | US MUTCD 6C.08 (work zones) [S1] | longer tapers "not necessarily better" in short urban blocks |
| Downstream taper | ~30 (100 ft) per lane | m | US MUTCD 6C.08 [S1] | |
| Lane-drop marking ahead of exit | wide dotted line from >= 0.8 km (1/2 mile) before gore | km | US MUTCD 3B.04 [S2] | freeway exit lane drops |
| Turn-bay taper | 10:1 (storage), 15:1 (deceleration) | ratio | US Iowa DOT [S79] (sec.) | |
| Turn-bay taper, urban | 15 - 30 (50 - 100 ft); suburban/rural 30 - 55 m | m | US (sec., study guide) | (unverified against AASHTO) |
| Left-turn storage minimum | 30 (100 ft) | m | US TxDOT [S80] (sec.) | >= 45 mph design speed |
| Cross-section width change | 1:25 (50 km/h), 1:30 (60), 1:35 (70), 1:45 (85), 1:50 (100), 1:55 (120) | ratio | UK CD 127 Table 2.28 [S17] | mainline and connector roads |
| Right-turn lane: direct taper | 5 (50-60 km/h), 15 (70-85), 25 (100), 30 (120) | m | UK CD 123 Table 5.22 [S19] | |
| Right-turn lane: deceleration length | 25 (50-60 km/h), 40 (70), 55 (85), 80 (100), 110 (120) | m | UK CD 123 Table 5.22 [S19] | level or 0-4 % grade |
| Right-turn lane: turning length | >= 10, plus queue length | m | UK CD 123 6.7 [S19] | |
| Urban turn bay, total | ~40 (5 + 25 + 10) | m | computed from UK CD 123 [S19] | 50 km/h |
| Corner taper after kerb radius | 1:5 over 30 m (urban), 1:10 over 25 m (rural) | ratio, m | UK CD 123 5.6.2 [S19] | swept path of large vehicles |
| Slip-lane taper | 1:10 (<= 60 km/h), 1:15 (> 60 km/h) | ratio | UK CD 116 6.32 [S21] | segregated left-turn lanes |
| Motorway merge entry taper / nose | rural mainline 205 m, nose >= 1:40, 115 m; urban <= 50 mph 75 m, 1:12, 40 m | m | UK CD 122 Table 3.21 [S20] | |
| Motorway diverge exit taper / nose | rural mainline 170 (1 lane) / 185 (2 lanes) m, 1:15, 80 m; urban <= 50 mph 75/90 m, 1:12, 40 m | m | UK CD 122 Table 3.32 [S20] | |
| Auxiliary lane length | merge 230 (rural motorway) to 100 (urban); diverge 200 to 80 | m | UK CD 122 [S20] | |
| Hatched nose width | <= 8 | m | UK CD 122 / TSRGD [S20] | |
| Additional lane | 3.65 m if > 100 m long; shorter ones per junction standard | m | UK CD 127 2.18 [S17] | |

OSM marks a lane that ends with `turn:lanes=...|merge_to_left` (or `merge_to_right`) on the lane that
disappears; values run from the leftmost to the rightmost lane in the way's direction [S56]. Where a
lane is added on one side, `placement=*` says where the OSM way sits relative to the road, so a lateral
shift of the generated reference line at a lane change can be a tagging artefact rather than real
geometry [S57].

**Judge as defect if**
- A through lane ends with zero taper (a width step) on a road at or above 50 km/h, or its taper is
  shorter than about half the MUTCD value for the posted speed.
- A lane "ends" in the middle of a block with no merge, no turn, no parking or bus use and no
  `merge_to_*` in the source data. That is usually a converter split.
- A lane appears and disappears again within < 15 m (a sliver lane) with no junction.

**Legitimate if**
- Urban turn bays are short: 25 - 40 m in total at 50 km/h (UK) and 30 m of storage (US) are normal,
  and tapers of 5 m (UK direct taper at 50 - 60 km/h) are allowed.
- An entry lane on a motorway continues as an auxiliary lane and drops at the next exit (lane gain /
  lane drop, UK CD 122). That is a design pattern, not a missing merge.
- A lane opens at the start of a bus lane, a parking strip or a turn bay with a short taper.

---

## 3. Turning radii

| Element | Value | Unit | Region / standard | Condition |
|---|---|---|---|---|
| Passenger car (P), min design turning radius | 7.25 (23.8 ft); inner 4.39 | m | US AASHTO via TxDOT [S11] | |
| Single-unit truck (SU-30) | 12.74 (41.8 ft); inner 8.66 | m | US AASHTO via TxDOT [S11] | |
| Semitrailer (WB-62) | 13.66 (44.8 ft); inner 2.26 | m | US AASHTO via TxDOT [S11] | |
| Centreline turning radius | P 6.4 (21 ft); SU-30 11.6 (38 ft); WB-67 12.5 (41 ft) | m | US AASHTO 2011 via Iowa DOT [S12] (sec.) | |
| 3-axle refuse truck | outer radius 10.25 (diameter 20.5); field practice needs >= 25 m diameter | m | DE Nürnberg waste authority [S44] (sec.) | add ~1 m overhang |
| Urban corner kerb radius | 3 - 4.5 (10 - 15 ft) standard; down to 0.6 (2 ft) | m | US NACTO [S5] | > 4.5 m (15 ft) is the exception |
| Actual kerb radius | 1.5 - 3 (5 - 10 ft) where possible | m | US FHWA PEDSAFE [S6] | |
| Effective turning radius | 4.5 - 6 (15 - 20 ft) urban, high pedestrian use; 7.5 - 9 (25 - 30 ft) bus/truck arterials; max 10.7 (35 ft) | m | US FHWA PEDSAFE [S6] | parking or bike lanes enlarge it |
| Corner radius, no design vehicle | 6 urban, 10 rural | m | UK CD 123 5.6.1 [S19] | simple priority junction |
| Corner radius with design vehicle | 10 urban (+1:5 taper), 15 rural (+1:10 taper) | m | UK CD 123 5.6.2 [S19] | |
| Corner radius, dual carriageway | 20 diverge; 20 - 30 merge; 40 after a diverge taper > 85 km/h | m | UK CD 123 5.6.5-5.7 [S19] | |
| Min curve radius (standard / reduced) | 120 km/h 710/570; 100: 460/380; 80: 280/230; 60: 150/120; 50: 100/80; 40: 60/50; 30: 30; 20: 15 | m | JP RSO Art. 15 [S26] | |
| Desirable min radius, e = 5 % | 120 km/h 1020; 100: 720; 85: 510; 70: 360; 60: 255; 50: 180 | m | UK CD 109 Table 2.10 [S18] | |
| Two steps below desirable, e = 7 % | 120 km/h 510; 100: 360; 85: 255; 70: 180; 60: 127; 50: 90 | m | UK CD 109 Table 2.10 [S18] | |
| Low-speed urban, normal crown (e = -2 %) | 15 mph 15; 20: 33; 25: 60; 30: 101; 35: 155; 45: 317 | m | US TxDOT (AASHTO method 2) [S10] | e = +4 %: 25 mph 47 m, 30 mph 76 m |
| Radius from speed | R = V^2 / (127 (q + f_t)) | m (V km/h) | IT DM 2001 5.2.4 [S32] | f_t 0.21 at 40 km/h to 0.11 at 100 km/h (A, B, C, F extra-urban) |
| Mountain hairpins, Stelvio | 48 numbered hairpins (Prato side); 34 (Bormio side) | count | climbing data [S33] (sec.) | |
| Hairpin inner radius | 14 (12 absolute); widened carriageway | m | IN IRC 52 (sec.) [S83] | the only numeric hairpin rule found; Alpine values (unverified) |
| Stelvio vehicle limits | width <= 3.30 m (Bormio side), length <= 10.50 m (Trafoi side) | m | Bormio tourism [S34] (sec.) | |
| Italian mountain roads | excluded from DM 2001 geometric rules | - | IT DM 5/11/2001 ch. 1 [S32] | "terreni morfologicamente difficili" |

**Judge as defect if**
- A junction connecting road has a centreline radius < 5 m for a car movement, or < 10 m on a route
  that buses or lorries must use, unless the corner is a known tight old-town corner.
- A curve on a motorway or expressway main line has a radius far below the table (for example < 250 m
  at 100 km/h), unless it is a ramp or loop.
- A U-turn or hairpin path is drawn with a radius a car cannot drive (< 5 m centreline). A hairpin
  that cars use needs a centreline radius of roughly 6.5 m or more (P design vehicle, [S12]).

**Legitimate if**
- Stelvio-type hairpins have centreline radii of roughly 8 - 15 m (unverified) with local widening and
  steep approaches. The Italian geometric standard explicitly does not apply to such mountain roads [S32].
- Urban kerb radii are 2 - 6 m and lorries sweep the opposite lane or a mountable apron.
- Motorway loop ramps have radii of a few tens of metres (design speed 40 - 60 km/h).

---

## 4. Junction design

| Element | Value | Unit | Region / standard | Condition |
|---|---|---|---|---|
| Roundabout, normal, inscribed circle diameter (ICD) | 28 - 100 | m | UK CD 116 3.5 [S21] | |
| Compact roundabout ICD | 28 - 36; circulatory width <= 6 | m | UK CD 116 [S21] | |
| Mini-roundabout ICD | <= 28; painted centre 1 - 4 m; 3 or 4 arms | m | UK CD 116 [S21] | <= 30 mph, 85th pct < 35 mph |
| Circulatory width | 1.0 - 1.2 x max entry width; normal <= 15 | m | UK CD 116 3.6 [S21] | |
| Entry path radius | <= 100 (normal); <= 70 (compact urban <= 40 mph) | m | UK CD 116 3.24-3.26 [S21] | deflection forces low speed |
| Entry angle | 20 - 60 | deg | UK CD 116 3.18 [S21] | |
| Central island | >= 4 | m diameter | UK CD 116 3.7 [S21] | normal and compact |
| Roundabout ICD | single-lane 32 - 46 (105 - 150 ft); multi-lane 46 - 91 (150 - 300 ft) | m | US NCHRP 672 via SCDOT [S78] (sec.) | |
| Mini roundabout (DE) | 13 - 22 outer diameter, ring 4 - 6, mountable centre | m | DE FGSV Merkblatt via Hessen [S43] (sec.) | |
| Small roundabout (DE) | 26 - 40 (up to ~50) outer; ring 9.0 at 26 m to 6.5 at 40 m | m | DE via Hessen and RASt tables [S43] (sec.) | |
| Splitter island (SDI), physical | >= 1.6 wide, >= 5 long, >= 2.4 from give-way line | m | UK CD 116 7.6-7.9 [S21] | painted >= 1.0 wide |
| Slip lane island side | >= 4.6 (15 ft); 3.7 (12 ft) constrained | m | US TxDOT 13.9 [S13] | |
| Slip lane entry angle ("smart right") | 70 recommended, >= 55 | deg | US TxDOT 13.9 [S13] | |
| Slip lane crosswalk | near middle of island, >= 6 m (20 ft) from yield line | m | US TxDOT 13.9 [S13] | |
| Stagger distance (right/left stagger) | >= 50 (ghost island or none), 40 (single-lane dualling), 60 (dual) | m | UK CD 123 2.25 [S19] | left/right stagger 50 - 130 m by speed |
| Crossroads vs stagger | minor-road centrelines that line up within the opposite carriageway = crossroads | - | UK CD 123 2.20 [S19] | |
| Weaving section, rural minimum | 2 km motorway; 1 km all-purpose | km | UK CD 122 4.5 [S20] | urban from design speed and flows |
| Interchange spacing | ~1.6 km (1 mile) rule of thumb | km | US (sec.) [S68] | |
| Fork in interchange link | taper 75 (1 lane) / 90 (2 lanes), nose 1:12, 40 m | m | UK CD 122 Table 3.38 [S20] | link design speed 70/85 km/h |
| Dual left-turn lanes | ~1.8x the capacity of one lane | - | US VTRC report 94-TAR7 (sec.) [S81] | skip-line guidance markings through the junction recommended |

Interchange vocabulary [S68][S69]: a **cloverleaf** turns left movements into loop ramps and creates
weaving between adjacent loops, often moved onto a **collector-distributor (C-D) road** that runs
parallel to the main line. **Braided ramps** cross each other on a grade separation to avoid weaving
between an on-ramp and the next off-ramp. **Directional** interchanges (four levels, like the
Frankfurter Kreuz) avoid weaving altogether.

How turn paths may cross:
- Opposing turns across traffic (left in RHT, right in LHT) may pass "offside to offside" (each turns
  behind the other) or "nearside to nearside" (in front of each other). The UK Highway Code calls the
  first generally safer [S25]. Both shapes are legitimate. Opposing left-turn lanes may be aligned
  head-to-head or offset (US Wisconsin DOT) [S82].
- Left-hand traffic (GB, JP) mirrors everything: the near-side turn is the left turn, the turn across
  traffic is the right turn, roundabouts give way to traffic from the right and circulate clockwise
  [S25], and slip lanes, turn bays and ghost islands sit on the mirrored side.
- Tokyo's Shuto Expressway has some merges and diverges on the right (passing-lane) side at
  route-to-route junctions such as Hakozaki [S72]. Ordinary on-ramps join from the left.

**Judge as defect if**
- Two connecting roads from the *same* approach cross each other inside the junction (for example a
  left-turn lane path crossing the through path of a lane to its right). Paths of one approach should
  fan out without crossing [S67].
- A turn path ends in an oncoming lane, a sidewalk, a median or a lane of the wrong direction.
- A roundabout circulates the wrong way for its traffic side, or an arm's connecting roads skip the ring.
- A motorway exit leaves from the median side where the source data shows a normal near-side exit.

**Legitimate if**
- Turn paths of *different* approaches cross (opposing left turns, a right turn across a through flow);
  conflicts are resolved by signal phases or priority, not geometry.
- Staggered junctions put two T-junctions 40 - 130 m apart; a short offset between them is a feature.
- A free-flow slip lane bypasses the junction behind a triangular island, or a mini-roundabout has a
  painted, overrunnable centre.
- An interchange has loops, braids, C-D roads, forks and left-side ramps (US HOV, Shuto right-side merges).

---

## 5. Pedestrians and wheelchair users

| Element | Value | Unit | Region / standard | Condition |
|---|---|---|---|---|
| Footway width | 2.0 normal min; 1.5 constrained; 1.0 at obstacles for <= 6 m | m | UK Inclusive Mobility 4.2 [S22] | 2.0 lets two wheelchairs pass |
| Headroom over footway | >= 2.3 | m | UK Inclusive Mobility [S22] | |
| Footway width | 2.50 standard (1.80 two walkers + 0.20 to wall + 0.50 to traffic) | m | DE RASt 06 / EFA via FUSS e.V. [S36] | 2.10 in residential streets; 1.50 in tight village main streets |
| Footway width | >= 1.50; >= 2.00 at kiosks etc. | m | IT DM 2001 3.4.6 [S32] | |
| Sidewalk width | > 3.5 busy roads; > 2.0 others (+0.5 to +3 m for trees, benches) | m | JP RSO Art. 11 [S26] | |
| Pedestrian access route | 1.22 (48 in) clear | m | US PROWAG R302.2 [S15] | |
| Passing space | 1.525 x 1.525 every <= 61 m (200 ft) | m | US PROWAG R302.3 [S15] | where route < 1.525 m wide |
| Running slope | <= 5 % or the street grade | % | US PROWAG R302.4 [S15] | |
| Cross slope | <= 2.1 % (1:48); 5 % in crosswalks at signals / uncontrolled | % | US PROWAG R302.5 [S15] | |
| Kerb (curb) ramp | <= 8.3 % (1:12), need not exceed 4.6 m (15 ft); flares <= 10 % | % | US PROWAG R304 [S15] | landing 1.22 x 1.22 m at <= 2.1 % |
| Blended transition | <= 5 % | % | US PROWAG R304.4 [S15] | |
| Detectable warning depth | >= 0.61 (24 in), full ramp width | m | US PROWAG R305 [S15] | >= 0.61 m apart on refuge islands |
| Path through median / refuge | >= 1.525 (60 in) wide | m | US PROWAG R302.2.1 [S15] | island 1.8 m long (6 ft) (sec.) |
| Gradients | 1:20 preferred; 1:12 absolute max (short ramps) | ratio | UK Inclusive Mobility [S22] | crossfall 1:50 preferred, 1:40 max |
| Dropped kerb | flush, <= 6 mm upstand; approach <= 1:12 (1:20 if space); flush width >= 1.2 (3.0 busy); = crossing width (>= 2.4) at controlled crossings | m | UK Inclusive Mobility [S22] | not inside the radius of an uncontrolled junction |
| Central refuge | >= 1.5, preferably 2.0 | m | UK Inclusive Mobility [S22] | >= 2.0 m between guardrails in staggered islands |
| Refuge on slip lane island | >= 2.0 | m | UK CD 116 8.4 [S21] | cycle refuge >= 3.0 m |
| Refuge island (DE) | 2.00 - 2.50 | m | DE RASt 06 (sec., municipal documents) | 2.50 for bicycles and wheelchairs |
| Crossing kerb (DE) | 3 cm shared kerb (r = 20 mm), or 0 cm flush 1 m wide + 6 cm kerb | cm | DE DIN 18040-3 via Bremen [S41] (sec.) | |
| Crosswalk width | >= 1.83 (6 ft) | m | US MUTCD 3B.18 [S2] | |
| Stop line before crosswalk | >= 1.22 (4 ft); without crosswalk 1.2 - 9.1 (4 - 30 ft) from the crossing road | m | US MUTCD 3B.16 [S2] | |
| Signal crossing width | 2.4 standard | m | UK Inclusive Mobility [S22] | |
| Crossing spacing | 80 - 100 urban; avoid > 200 | m | Global Street Design Guide [S48] | NZTA: > 3 min detour invites unsafe crossing [S49] |
| Crossing spacing | 160 (530 ft) in pedestrian districts, 245 (800 ft) elsewhere | m | US Portland (sec.) [S49] | |
| Marked crosswalk, unsignalled junction (JP) | carriageway about >= 3.5 wide (mid-block >= 5.5, newspaper report) | m | JP National Police Agency criteria via Osaka City [S77] (sec.) | |
| Zebra (DE "Fussgängerüberweg") | only <= 50 km/h, one lane per direction, footway on both sides; carriageway <= 6.50, island preferred if >= 8.50 | m | DE R-FGÜ 2001 (sec., municipal documents) [S42] | |
| Crossings at roundabouts | signal crossings 20 m or > 60 m from give-way line; not 20 - 60 m | m | UK CD 116 3.59, 8.2 [S21] | |
| Kerb height | >= 100 mm urban with footway; 75 mm rural, footway > 1.3 m from edge | mm | UK CD 127 [S17] | no kerbs where no footway (rural) |
| Kerb height | vertical 150 - 225 (Type A); mountable <= 100 - 150 | mm | US AASHTO via NAP / TxDOT [S73] | |
| Kerb height | ~120 typical; 30 lowered | mm | DE (sec., OSM wiki) [S74] | bus stop kerbs 160 - 240 (sec.) |

Usage. In a UK stated-preference study about half of the respondents never chose to cross away from a
crossing; the rest crossed only when the detour exceeded roughly 7.5 - 8 minutes [S50]. Pedestrians
follow desire lines, so short-cuts across a road near a crossing are common. In Japan pedestrians walk on
the **right** where there is no sidewalk, and painted roadside strips act as footways [S30].

Mapping. OSM `sidewalk=both|left|right|no|separate` describes the street; `separate` means the
footway is its own way tagged `footway=sidewalk`; left and right follow the way's direction [S53].
A `highway=crossing` node sits on the road and the footway (or `footway=crossing` way) joins that same
node; `crossing=traffic_signals|uncontrolled|marked|unmarked` and `crossing:island=yes` describe it [S54].

**Judge as defect if**
- A crosswalk ends in a driving lane, a parking lane or a median that has no refuge area; it must end
  on a footway, a refuge island, a shared surface or a kerb ramp on both sides.
- A sidewalk stops abruptly in an urban block with no crossing, driveway, building line or tag change,
  or alternates between sides at random within a block.
- A sidewalk is narrower than ~1.0 m over a long stretch, or a refuge island used by the crossing is
  < 1.2 m deep (a wheelchair or pram does not fit).
- A crossing has no lowered kerb or ramp on one end (wheelchair user stranded), unless the source tags
  say `kerb=raised` there.

**Legitimate if**
- Sidewalks are absent on motorways, expressways, ramps, rural roads, mountain passes (Stelvio),
  tunnels without walkways, service roads, and in shared spaces / living streets where walkers may use
  the whole width [S58].
- A sidewalk ends where the footway leaves as a separate path, at a dead end, at a driveway
  (it continues level across it), or where a crossing takes it to the other side.
- Unmarked crossings exist at intersections, refuges are staggered, and Shibuya-style scramble crossings
  have diagonal crosswalks inside the junction.

---

## 6. Cyclists

| Element | Value | Unit | Region / standard | Condition |
|---|---|---|---|---|
| Advisory lane (Schutzstreifen) | 1.50 (was 1.25) | m | DE RASt 06 supplement 2024 [S35] | |
| Cycle lane (Radfahrstreifen) | 2.00 + wide line (was 1.60 - 1.85) | m | DE FGSV 2024 [S35]; ERA 2010 1.85 [S38] | |
| Cycle track (Radweg) | 2.00 (was 1.60) | m | DE FGSV 2024 [S35] | |
| Buffer to parking | >= 0.75 (was 0.50) | m | DE FGSV 2024 [S35] | |
| Overtaking distance car -> cyclist | >= 1.50 | m | DE StVO via FGSV [S35] | a 3.75 m carriageway with advisory lane is too narrow |
| Cycle lane / track | 2.0 desirable min; 1.5 absolute min at constraints | m | UK LTN 1/20 Table 5-2 via Hampshire [S23][S24] (sec.) | |
| Bike lane | 1.83 (6 ft) desirable; 1.52 (5 ft) | m | US NACTO [S9] (sec.) | rideable surface beside gutter >= 0.9 m |
| Bike lane + parking reach | 4.42 (14.5 ft) desirable; 3.66 (12 ft) min | m | US NACTO via Sebastopol (sec.) | |
| Bicycle track | >= 2.0 (1.5 constrained) | m | JP RSO as adopted by Osaka [S27] | |
| Cycle path | 2.5 one-way, 3.5 two-way | m | NL CROW (sec., Wikipedia) | (unverified against CROW) |
| Advanced stop line reservoir | 5 deep (original DfT layouts); 3 - 5 (Southwark) | m | UK DfT ASL guidance, Southwark DS 303 (sec.) | lead-in lane >= 1.5 m |
| Cyclist refuge on islands | >= 3.0 | m | UK CD 116 8.10 [S21] | |
| Protected corner island | keep turning speed < 16 km/h (10 mph) | km/h | US NACTO (sec.) | mountable truck apron allowed |

Usage and rules. Japanese cyclists ride on the left and must turn right in two stages (straight across,
then turn and wait) [S29]. German cyclists may use a cyclist-only green arrow [S71]. In Germany and the
UK cycle infrastructure often switches between track, lane and mixed traffic along one street.

**Judge as defect if**
- A cycle lane ends and drops the cyclist into the middle of a junction or into an oncoming lane, or
  leads straight into a kerb or bus stop platform with no transition.
- A cycle lane is drawn between two general traffic lanes for a long distance without being a marked
  lead-in to an ASL or a turn-lane configuration.
- A cycle track crossing a side road is not connected to the cycle track on the other side.

**Legitimate if**
- Cycle lanes end at a junction approach, become an ASL lead-in lane, merge into mixed traffic on
  20 - 30 km/h streets, or ramp up onto a track; short gaps at bus stops and pinch points occur.
- Bicycles use bus lanes (shared bus-bike lanes) or contra-flow lanes in one-way streets.
- Protected intersections bend the cycle track away from the corner (setback), giving S-shaped paths.

---

## 7. Dead ends and connectivity

| Element | Value | Unit | Region / standard | Condition |
|---|---|---|---|---|
| Dead-end nodes, median US neighbourhood | 14.5 | % of nodes | US OSMnx study of 27,000 networks [S46] | 3-way 57.4 %, 4-way 23.4 % |
| Dead-end nodes, typical urbanised area | ~21; > 30 in Atlanta, Chattanooga | % of nodes | US Boeing [S46][S47] | |
| Trend | dead-end share 163 % higher in 1990s tracts than pre-1940; lower again after 2000 | - | US Boeing [S47] | |
| Fire access dead end needing a turnaround | > 46 (150 ft) | m | US IFC App. D (as adopted locally) [S45] (sec.) | width 6.1 m (20 ft) for 46 - 152 m |
| Turnaround size | cul-de-sac 29 (96 ft) diameter; hammerhead 37 (120 ft); "Y" 18 (60 ft) | m | US IFC App. D [S45] (sec.) | |
| Refuse truck turning circle | 20.5 - 25 diameter; hammerhead with <= 2 reversing moves | m | DE waste authorities [S44] (sec.) | |
| Fire access width | >= 3.0 clear (straight), height >= 3.5; 5.0 at 10.5 - 12 m outer radius | m | DE Bavarian fire-area guideline [S75] (sec.) | DIN 14090 revised 2024 |
| Fire tender operating width | >= 3.7 between kerbs | m | UK Approved Doc. B (sec.) [S84] | |
| Minimum "road" for building access | 4 m wide (narrower pre-1950 roads exist with set-back) | m | JP Building Standards Act Art. 42 [S31] | |

Mapping. OSM `noexit=yes` on the last node confirms an intentional dead end; it is not an access
restriction and routers ignore it [S51]. `highway=turning_circle` marks a widened turning area, usually
at the end of a dead-end street [S52]. CARLA warns that roads generated from OSM end abruptly at the
map border, which can crash its Traffic Manager; its OSM mode destroys vehicles that reach such ends [S62].

**Judge as defect if**
- A public road ends a few metres short of another road, a junction or a bridge with no `noexit`,
  barrier, level difference or turning area (a missing link).
- A lane leads nowhere inside the network: a driving lane with no successor that is not at the map
  border, a turning area, a parking aisle, a driveway or a barrier.
- One carriageway of a dual carriageway ends while the other continues, or a one-way street ends
  without a junction (traffic is trapped).
- The network splits into islands that real streets connect.

**Legitimate if**
- Cul-de-sacs (with or without turning heads), hammerheads, service roads, parking aisles, driveways,
  gated or private roads, roads cut by bollards (filtered permeability), and footpath-only continuations.
- Roads end at the edge of the extracted map area (expected; a simulator needs to handle or trim them).
- Dead-end shares of 10 - 30 % of nodes in US urban and suburban areas [S46].

---

## 8. Vertical alignment, cross slope and stacking

| Element | Value | Unit | Region / standard | Condition |
|---|---|---|---|---|
| Max grade (standard / reduced) | 120 km/h 2/5; 100: 3/6; 80: 4/7; 60: 5/8; 50: 6/9; 40: 7/10; 30: 8/11; 20: 9/12 | % | JP RSO Art. 20, Types 1-3 [S26] | Type 4 urban: 5/7 at 60 to 9/11 at 20 km/h |
| Max grade (desirable / relaxation) | motorway 3/4; AP dual 4/8; AP single 6/8 | % | UK CD 109 Table 5.1 [S18] | min 0.5 % on kerbed roads |
| Max grade | A 5 (rural) / 6 (urban); B 6; C 7; D 6; E 8; F 10 | % | IT DM 2001 5.3 [S32] | +1 % allowed after checking |
| Max grade, Interstate | ~6 (more in mountains) | % | US (sec.) [S76] | |
| Mountain pass gradients | Stelvio from Prato: 24.7 km, +1843 m, avg 7.5 %, steepest 100 m 10.6 % | % | [S33] (sec.) | single GPS peaks of 14 % reported |
| Grade on a hairpin | flattened to ~2.5 - 4 | % | IN IRC (sec.) [S83] | (unverified for the Alps) |
| Max superelevation | 10 (normal), 8 (snowy), 6 (very snowy); Type 4 urban 6 | % | JP RSO Art. 16 [S26] | |
| Max superelevation | 7 rural; 5 urban | % | UK CD 109 4.3, 4.6 [S18] | |
| Max superelevation | 7 (A, B, C, F rural); 5 (D); 3.5 (E, F urban) | % | IT DM 2001 5.2.4 [S32] | |
| Max superelevation | 6 typical, 8 allowed; low-speed urban 4 (normal crown preferred) | % | US TxDOT [S10] | |
| Normal cross slope | >= 2.5 | % | IT DM 2001 5.2.3 [S32] | |
| Normal cross slope | 1.5 - 2 (paved) | % | JP RSO via Osaka [S27] | |
| Combined grade + crossfall | <= 10 (A, B); <= 12 (others); <= 8 (frequent snow) | % | IT DM 2001 [S32] | |
| Roundabout circulatory crossfall | 2 typical; <= 2.5 | % | UK CD 116 3.40 [S21] | |
| Vertical clearance | 4.5 (3.0 small-car roads; 4.0 exception) | m | JP RSO Art. 12 [S26] | |
| Headroom | 5.03 + S (general), 5.30 + S (overbridges) | m | UK CD 127 [S17] | S = sag allowance |
| Clearance over Interstate | 4.9 - 5.0 (16 - 16.5 ft); 4.3 (14 ft) urban (sec.) | m | US (sec.) [S76] | |

**Judge as defect if**
- A street grade exceeds ~12 % for more than a short ramp, or a motorway grade exceeds ~6 %.
- Elevation jumps by more than a few centimetres at a road link (a step in the surface).
- Two crossing roads in plan have a vertical separation smaller than the clearance plus a structure
  depth (roughly < 5 - 6 m between surfaces; structure depth (unverified)) yet are not a junction.
- Superelevation is > 10 %, tilts the wrong way on a curve, or flips sign without a transition.

**Legitimate if**
- Mountain passes climb at 7 - 8 % average with short pitches above 10 % (Stelvio [S33]); hairpins
  themselves are usually flatter (IRC rule [S83]; Alpine practice (unverified)).
- Urban streets cross on bridges and in tunnels; Tokyo has elevated expressways stacked above streets
  and the Frankfurter Kreuz has four levels. Overlap in plan is fine when the levels are separated.
- Low-speed urban streets have normal crown (no superelevation) even on curves.

---

## 9. Traffic rules that change what is legitimate

| Rule | Value / statement | Region | Source |
|---|---|---|---|
| Traffic side | right-hand traffic: DE, IT, US; left-hand traffic: GB, JP | - | general; CARLA `rule="LHT"` per road [S60] |
| Turn on red | right on red after a stop unless signed; left on red only one-way to one-way | US (CA) | CA Vehicle Code 21453 [S70] |
| Turn on red | right turn on red only with the green-arrow sign, after stopping | DE | StVO §37 [S71] |
| Turn on red | not allowed; only on a green filter arrow | GB | Highway Code r.177 [S25] |
| Turn on red | left on red only with a sign or green arrow | JP | JAF [S29] |
| U-turn | allowed unless signed; also on a right-turn arrow | JP | JAF [S29] |
| U-turn | avoid on busy roads and at mini-roundabouts | GB | Highway Code r.188, r.200 [S25] |
| Box junction | do not enter unless the exit is clear | GB | Highway Code r.174 [S25] |
| Pedestrians at side roads | give way to people crossing the road you turn into | GB | Highway Code r.170 [S25] |
| Turn restrictions in OSM | `type=restriction`, `no_*` / `only_*`, from / via (node or ways) / to, `except=` | world | OSM wiki [S55] |
| Implicit rules | country default rules (for example unsigned U-turn bans) are not mapped | world | OSM wiki [S55] |
| Bicycles turning across traffic | two-stage turn required | JP | JAF, police guides [S29] |
| Tram vehicle | width <= 2.65 m (up to 3.4 m height); curve allowance <= 0.65 m per side | DE | BOStrab §34 [S40] |
| Tram safety space | >= 0.7 m wide, 2.0 m high beside the swept envelope | DE | BOStrab §19 [S39] |
| Tram track centres | 3.20 m (plain track), 3.60 m at a stop (Bremen example) | DE | (sec.) |
| Living street / shared space | walking pace up to 20 km/h; pedestrians have priority; no kerbed sidewalk needed | world | OSM wiki [S58] |

**Judge as defect if**
- A junction offers a movement that a mapped `no_*` restriction forbids, or omits the only allowed
  movement of an `only_*` restriction.
- A one-way street gets lanes in both directions, or a turn path leads against a one-way.
- Driving lanes of an LHT city (London, Tokyo) have right-hand traffic orientation, or CARLA receives
  them without `rule="LHT"`.

**Legitimate if**
- Turn paths exist for movements that are only legal on a green arrow (they are signal-controlled).
- Trams run in the general traffic lane (street-running track) or on a kerbed reservation; bus and tram
  lanes start and stop with short tapers at stops and junctions.
- Contra-flow bus or cycle lanes run against a one-way street.

---

## 10. Simulators: what CARLA and esmini need

| Item | Value / behaviour | Source |
|---|---|---|
| OpenDRIVE semantics | CARLA's map API follows OpenDRIVE 1.4 road / lane / junction concepts | CARLA maps [S60] |
| Mesh generation parameters | vertex_distance 2.0 m; max_road_length 50 m; wall_height 1.0 m; additional_width 0.6 m on junction lanes; smooth_junctions on | CARLA standalone mode [S59] |
| Known limits | lateral slope not integrated; one fixed sidewalk height; junction smoothing alters geometry; file errors propagate, especially at junctions | CARLA [S59] |
| Waypoints | 3D directed points per lane; topology = lane start/end pairs; ids collapse within 2 cm | CARLA [S60] |
| Spawn points | suggested by the map; may collide with existing actors | CARLA [S60] |
| Traffic Manager | paths built from connected waypoints; random direction at junctions; paths are endless; priority not by traffic law; roundabouts are a known weak spot | CARLA TM [S61] |
| Map borders | abruptly ending OSM roads can crash the TM; OSM mode warns and destroys vehicles | CARLA OSM [S62] |
| Traffic lights / stops / yields | generated from OpenDRIVE signals | CARLA [S59][S60] |
| Pedestrians | navigate on sidewalks and crosswalks (navigation built from meshes named Road_Sidewalk / Road_Crosswalk); OpenDRIVE objects of type crosswalk are parsed | CARLA [S63] |
| esmini supported | road links, planView incl. paramPoly3, elevation, superelevation, laneOffset, lane width, roadMark, objects, signals, junction types default and direct | esmini coverage [S64] |
| esmini unsupported | virtual junctions, junction priority / controller, lane height / speed / access / rule, crossfall and shape, surface, junction groups, signal references, railroad elements | esmini coverage [S64] |
| esmini odrviewer | draws the network and drives random dummy traffic through it | esmini [S65] |
| OpenDRIVE linkage rules | lanes without a laneLink have no path through a junction; connecting roads per lane movement; no lane change inside a junction for one-link roads; linked lanes fit smoothly | ASAM OpenDRIVE 1.9 via repo note [S66][S67] |

**Judge as defect (for simulation) if**
- A driving lane has no successor or predecessor where the road continues, or a junction incoming lane
  has no laneLink (vehicles stop or vanish there).
- Connecting roads leave or join lanes of type sidewalk, border, parking or median.
- Elevation or heading jumps at links (vehicles bounce or fall through), or sidewalks have no
  height / type that CARLA can turn into walkable areas.
- A roundabout or junction is modelled so that TM vehicles can loop forever with no exit.

**Legitimate if**
- Lanes end at the map border (expected), in parking aisles or at `noexit` ends, provided the reviewer
  notes that CARLA's Traffic Manager may need those vehicles removed.
- Junction meshes look different in CARLA because of `smooth_junctions`.
- Unsupported esmini elements (crossfall, lane height) are simply ignored by esmini.

---

## 11. Plan-view traps

**Looks wrong but is legitimate**
1. Short urban turn bays (25 - 40 m including taper at 50 km/h) and 5 m direct tapers [S19].
2. Staggered T-junctions 40 - 130 m apart instead of a crossroads [S19].
3. Bridges, tunnels and multi-level interchanges whose roads overlap in plan (Frankfurter Kreuz, Shuto).
4. Lane lines that kink or shift at a stop line (a lane widens into two short turn lanes, or a centre
   line shifts to make room for a turn bay).
5. Lanes that change width by 0.25 - 0.5 m between sections (3.25 to 3.0 m, 3.75 to 3.5 m).
6. Single-lane or 4 m old-town streets with no centre line and no sidewalks (JP "Article 42" roads,
   European old towns, shared spaces) [S31][S58].
7. Mountain hairpins with radii of ~10 m and steep approaches; switchback stacks on the Stelvio [S33].
8. Cul-de-sacs, hammerheads and bollard-filtered streets [S46][S51].
9. Right-side merges on Tokyo expressways at route junctions; left exits and HOV lanes in Los Angeles [S72].
10. Opposing turning paths that cross in the junction centre, or pass nearside to nearside [S25].
11. Roundabouts with mountable centres (mini-roundabouts), and slip lanes behind islands [S21].
12. Scramble crossings with diagonal crosswalks; refuges that offset the crosswalk (staggered crossing).
13. Bus lanes, bus gates and tram tracks in the general traffic lane.
14. Cycle lanes that end at a junction and continue after it; ASL boxes ahead of the stop line.
15. Sidewalks only on one side, or none, on narrow residential or rural roads; pedestrians on the
    right edge of the road in Japan [S30].

**Looks fine but is wrong**
1. A lane link that crosses a median, a kerbed island or a tram reservation (lanes linked across a
   physical separation).
2. A crosswalk whose end lies in a driving or parking lane instead of on a footway or refuge.
3. A turn connection from a through-only lane, or one that contradicts `turn:lanes` or a restriction [S55][S56].
4. Connecting roads of one approach that cross each other inside the junction [S67].
5. A lane that ends with a width step and no taper on a road at 50 km/h or more.
6. Two carriageways of a dual carriageway merged into one two-way road (median lost), or a one-way
   street rendered with two-way lanes.
7. A road that ends 1 - 10 m before another road with no barrier (missing link), or a short sliver lane.
8. A sidewalk lane that continues across a motorway ramp or through a junction without a crossing.
9. An LHT network (London, Tokyo) generated with right-hand traffic lane orientation.
10. Elevation steps at road links, or a bridge deck at the same height as the road below.
11. Lane widths that look right but are two lanes fused (5 - 7 m) or a parking lane typed as driving.
12. A junction with incoming lanes that have no laneLink (no path for simulator traffic) [S66].
13. A wheelchair route that meets a raised kerb at a crossing with no lowered kerb or ramp [S15][S22].

---

## Quick thresholds for the reviewer

The "Typical" column condenses the cited ranges. The "Flag" column is a reviewer heuristic derived from
them (roughly where a value leaves the range of anything built), not a value taken from a standard.

| Check | Typical | Flag below / above | Unit | Source |
|---|---|---|---|---|
| Urban lane width | 2.75 - 3.5 | < 2.5 or > 4.5 (unexplained) | m | [S3][S4][S27][S35] |
| Motorway lane width | 3.5 - 3.75 | < 3.25 | m | [S17][S37] |
| Turn lane width | 2.5 - 3.5 | < 2.5 | m | [S19][S27] |
| Bus lane width | 3.0 - 4.5 | < 2.75 (outside stops) | m | [S7][S85] |
| Lane-drop taper, < 45 mph | W*S^2/60 (ft) | < 0.5 x formula | ft | [S2] |
| Lane-drop taper, >= 45 mph | W*S (ft) | < 0.5 x formula | ft | [S2] |
| Width-change taper, 50 km/h | 1:25 | steeper than ~1:10 | ratio | [S17] |
| Urban turn bay length (50 km/h) | 25 - 40 | < 15 | m | [S19][S80] |
| Car turning centreline radius | >= 6.4 | < 5 | m | [S12] |
| Bus / lorry turning radius | 12 - 14 outer | path < 10 outer on a bus route | m | [S11] |
| Urban kerb radius | 1.5 - 6 | > 15 (urban, unexplained) | m | [S5][S6][S19] |
| Min radius at 50 km/h | 80 - 180 | < 80 | m | [S26][S18] |
| Min radius at 100 km/h | 380 - 720 | < 360 | m | [S26][S18] |
| Roundabout ICD (normal) | 28 - 100 | < 13 (except mini) | m | [S21][S43] |
| Weaving length, rural motorway | >= 2000 | < 1000 | m | [S20] |
| Footway width | 1.5 - 2.5 | < 1.0 (long stretch) | m | [S22][S36][S15] |
| Refuge island depth | 1.5 - 2.5 | < 1.2 | m | [S22][S15] |
| Kerb ramp slope | <= 8.3 | > 8.3 | % | [S15][S22] |
| Kerb height at crossing | 0 - 30 | > 60 with no ramp | mm | [S22][S41] |
| Crosswalk width | 1.8 - 4 | < 1.8 | m | [S2][S22] |
| Crossing spacing (urban) | 80 - 100 | > 200 | m | [S48] |
| Cycle lane width | 1.5 - 2.0 | < 1.25 | m | [S23][S35][S9] |
| Max grade, streets | 5 - 10 | > 12 | % | [S26][S32] |
| Max grade, motorway | 3 - 4 | > 6 | % | [S18][S26] |
| Superelevation | 2 - 7 | > 10 | % | [S26][S18][S32] |
| Vertical clearance under structures | 4.5 - 5.3 | < 4.0 (general roads) | m | [S26][S17] |
| Dead-end share of nodes | 10 - 30 | > 40 (check for broken links) | % | [S46] |
| Dead end needing a turnaround (US) | > 46 m long | none provided | m | [S45] |

---

## Sources

- [S1] FHWA MUTCD 2009, Part 6C (tapers, Tables 6C-3 / 6C-4): https://mutcd.fhwa.dot.gov/htm/2009/part6/part6c.htm
- [S2] FHWA MUTCD 2009, Part 3B (3B.04, 3B.09, 3B.16, 3B.18, 3B.20): https://mutcd.fhwa.dot.gov/htm/2009/part3/part3b.htm
- [S3] FHWA, lane width (AASHTO Green Book values): https://www.fhwa.dot.gov/planning/css/resources/lanewidth
- [S4] NACTO Urban Street Design Guide, lane width (read via search summaries; page returned 403): https://nacto.org/publication/urban-street-design-guide/street-design-elements/lane-width/
- [S5] NACTO Urban Street Design Guide, corner radii: https://nacto.org/publication/urban-street-design-guide/intersection-design-elements/corner-radii/
- [S6] FHWA PEDSAFE, countermeasure 28 (curb radius): https://pedbikesafe.org/PEDSAFE/countermeasures_detail.cfm?CM_NUM=28
- [S7] NACTO, dedicated curbside / offset bus lanes: https://nacto.org/publication/urban-street-design-guide/street-design-elements/transit-streets/dedicated-curbside-offset-bus-lanes/
- [S8] NACTO Transit Street Design Guide, offset transit lane: https://nacto.org/publication/transit-street-design-guide/transit-lanes-transitways/transit-lanes/offset-transit-lane
- [S9] NACTO, conventional bike lanes: https://nacto.org/treatments/bike-lanes/conventional-bike-lanes
- [S10] TxDOT Roadway Design Manual 4.7, horizontal alignment: https://www.txdot.gov/manuals/des/rdw/chapter-4--basic-design-criteria/4-7-horizontal-alignment.html
- [S11] TxDOT Roadway Design Manual 4.3.3, control vehicles: https://www.txdot.gov/manuals/des/rdw/chapter-4--basic-design-criteria/4-3-traffic-characteristics/4-3-3-control-vehicles.html
- [S12] Iowa DOT Design Manual 6A-2, design vehicles (AASHTO 2011): https://ia.iowadot.gov/design/dmanual/06a-02.pdf
- [S13] TxDOT Roadway Design Manual 13.9, right-turn slip lanes: https://www.txdot.gov/manuals/des/rdw/chapter-13--intersections/13-9-right-turn-slip-lanes.html
- [S14] Caltrans Highway Design Manual, Chapter 300: https://dot.ca.gov/-/media/dot-media/programs/design/documents/chp0300-032020.pdf
- [S15] US Access Board, PROWAG technical requirements (2023 final rule): https://www.access-board.gov/prowag/technical.html
- [S16] Potts, Harwood, Richard (2007), Relationship of Lane Width to Safety on Urban and Suburban Arterials: https://trid.trb.org/View/802590
- [S17] DMRB CD 127 Cross-sections and headrooms, v1.0.1: https://standardsforhighways.co.uk/tses/attachments/10442706-b592-42c8-85f8-2a0c779a8e37
- [S18] DMRB CD 109 Highway link design, Rev. 1 (council-hosted copy): https://www.northwarks.gov.uk/downloads/file/1364/cd-109-revision-1-highway-link-design-web-8-
- [S19] DMRB CD 123 At-grade priority and signal-controlled junctions, v2.1.0: https://www.standardsforhighways.co.uk/tses/attachments/962a81c1-abda-4424-96c9-fe4c2287308c
- [S20] DMRB CD 122 Grade separated junctions, v1.1.1: https://www.standardsforhighways.co.uk/tses/attachments/3ab9ef31-9880-4e8e-a7eb-f3d218e74ffd
- [S21] DMRB CD 116 Geometric design of roundabouts, v2.1.0 (HTML): https://standardsforhighways.co.uk/tses/attachments/html/7b5ea157-9b3e-4774-9781-7d1656e83338
- [S22] DfT Inclusive Mobility (2021): https://assets.publishing.service.gov.uk/government/uploads/system/uploads/attachment_data/file/1044542/inclusive-mobility-a-guide-to-best-practice-on-access-to-pedestrian-and-transport-infrastructure.pdf
- [S23] DfT LTN 1/20 Cycle infrastructure design (PDF too large to read here): https://www.gov.uk/government/publications/cycle-infrastructure-design-ltn-120
- [S24] Hampshire CC technical guidance quoting LTN 1/20 widths: https://www.testvalley.gov.uk/assets/attach/25506/11.03-Transport-Appendix-2-HCC-TG10-Extract.pdf
- [S25] The Highway Code, rules 159 - 203: https://www.gov.uk/guidance/the-highway-code/using-the-road-159-to-203
- [S26] MLIT Japan, Road Structure Ordinance technical standards (English): https://www.mlit.go.jp/road/road_e/q4_standard_2.html
- [S27] Osaka Prefecture road structure ordinance (adopts the national values): https://www.pref.osaka.lg.jp/houbun/reiki/reiki_honbun/k201RG00001790.html
- [S28] MLIT, Road Structure Ordinance explanation (lane widths; read via search summary): https://www.mlit.go.jp/road/sign/pdf/kouzourei_3.pdf
- [S29] JAF, traffic rules in Japan: https://english.jaf.or.jp/driving-in-japan/traffic-rules
- [S30] Aichi Prefecture, traffic rules for residents: https://www.pref.aichi.jp/global/en/living/traffic/index.html
- [S31] Yokohama City, roads under the Building Standards Act: https://translate-en.city.yokohama.lg.jp/business/bunyabetsu/kenchiku/annai/faq/douro.html
- [S32] Italy DM 5/11/2001 n. 6792, Norme funzionali e geometriche (provincial copy): https://www.provincia.cremona.it/strade/all/20111110-1156570.pdf
- [S33] Climbfinder, Passo dello Stelvio: https://climbfinder.com/en/climbs/passe-dello-stelvio
- [S34] Bormio tourism, Stelvio pass: https://www.bormio.eu/en/stelvio-pass
- [S35] FGSV, Ad-hoc-Arbeitspapier "Ergänzende Handlungsanleitungen zur Anwendung der RASt 06" (2024): https://www.fgsv-verlag.de/pub/media/pdf/26502.v.pdf
- [S36] FUSS e.V., Wie breit müssen Gehwege sein? (RASt 06 / EFA values): https://www.fuss-ev.de/wp-content/uploads/2024/12/gehwegbreiten_v3_Stand_2022-10-11_Download.pdf
- [S37] Deutscher Bundestag, Wissenschaftliche Dienste WD 5-084/22 (RQ 31): https://www.bundestag.de/resource/blob/905110/cfb962c0d0eeaf67f7e17198a8ca2dbc/WD-5-084-22-pdf.pdf
- [S38] Peters, ERA 2010 presentation: https://verkehrslexikon.de/PDF/Peters_ERA_2010.pdf
- [S39] BOStrab §19 (safety space): https://www.gesetze-im-internet.de/strabbo_1987/__19.html
- [S40] BOStrab §34 (vehicle width): https://www.gesetze-im-internet.de/strabbo_1987/__34.html
- [S41] Bremen disability commissioner on DIN 18040-3 kerbs: https://www.behindertenbeauftragter.bremen.de/sixcms/media.php/13/2018-05-16%20Stellungnahme%20.pdf
- [S42] Munich council document quoting R-FGÜ 2001 (zebra crossing criteria): https://www.muenchen-transparent.de/dokumente/9211293/datei
- [S43] Hessen, Leitfaden Qualitätssicherung Kreisverkehre: https://mobil.hessen.de/sites/mobil.hessen.de/files/2022-02/130723_leitfaden_qualitaetssicherung_kreisverkehre.pdf
- [S44] Nürnberg, Merkblatt Standplätze und Zufahrtswege (refuse trucks): https://www.nuernberg.de/imperia/md/asn/dokumente/merkblatt_standplaetze_zufahrtswege.pdf
- [S45] IFC Appendix D as adopted by Orono, MN (Table D103.4): https://publicinfo.ci.orono.mn.us/WebLink/0/doc/130341/Page71.aspx
- [S46] Boeing, A Multi-Scale Analysis of 27,000 Urban Street Networks: https://arxiv.org/pdf/1705.02198
- [S47] Boeing, Off the Grid... and Back Again?: https://arxiv.org/pdf/2010.04771
- [S48] Global Designing Cities Initiative, pedestrian crossings: https://globaldesigningcities.org/publication/global-street-design-guide/designing-streets-people/designing-for-pedestrians/pedestrian-crossings/
- [S49] NZTA, location and spacing of crossings: https://nzta.govt.nz/walking-cycling-and-public-transport/walking/walking-standards-and-guidelines/pedestrian-network-guidance/design/crossings/crossing-selection/location-and-spacing-of-crossings/
- [S50] Anciaes and Jones (UCL), pedestrian crossing choices: https://discovery.ucl.ac.uk/id/eprint/1505652/
- [S51] OSM wiki, noexit=yes: https://wiki.openstreetmap.org/wiki/Tag:noexit%3Dyes
- [S52] OSM wiki, highway=turning_circle: https://wiki.openstreetmap.org/wiki/Tag:highway=turning_circle
- [S53] OSM wiki, Key:sidewalk: https://wiki.openstreetmap.org/wiki/Key:sidewalk
- [S54] OSM wiki, highway=crossing: https://wiki.openstreetmap.org/wiki/Tag:highway%3Dcrossing
- [S55] OSM wiki, Relation:restriction: https://wiki.openstreetmap.org/wiki/Relation:restriction
- [S56] OSM wiki, Key:turn (turn:lanes): https://wiki.openstreetmap.org/wiki/Key:turn
- [S57] OSM wiki, Key:placement: https://wiki.openstreetmap.org/wiki/Key:placement
- [S58] OSM wiki, highway=living_street: https://wiki.openstreetmap.org/wiki/Tag:highway=living_street
- [S59] CARLA docs, OpenDRIVE standalone mode: https://carla.readthedocs.io/en/latest/adv_opendrive/
- [S60] CARLA docs, maps and navigation: https://carla.readthedocs.io/en/latest/core_map/
- [S61] CARLA docs, Traffic Manager: https://carla.readthedocs.io/en/latest/adv_traffic_manager/
- [S62] CARLA docs, generate maps with OpenStreetMap: https://carla.readthedocs.io/en/latest/tuto_G_openstreetmap/
- [S63] CARLA docs 0.9.14, generate pedestrian navigation: https://carla.readthedocs.io/en/0.9.14/tuto_M_generate_pedestrian_navigation/
- [S64] esmini OpenDRIVE coverage list: https://raw.githubusercontent.com/esmini/esmini/master/odr_coverage.txt
- [S65] esmini user guide, tools overview: https://esmini.github.io/tools-overview.html
- [S66] ASAM OpenDRIVE 1.9.0 specification: https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.9.0/specification/
- [S67] Repository note docs/reference/opendrive_junction_rules.md (OpenDRIVE junction rules restated, incl. 1.6 Quick Read figures)
- [S68] Interchange glossary quoting FHWA TFHRC (cloverleaf, C-D road, braiding): https://www.kurumi.com/roads/interchanges/gloss.html
- [S69] WSDOT Design Manual 1360, interchanges: https://wsdot.wa.gov/publications/manuals/fulltext/M22-01/1360.pdf
- [S70] California Vehicle Code 21453: https://law.justia.com/codes/california/code-veh/division-11/chapter-2/article-3/section-21453
- [S71] StVO §37 (Grünpfeil): https://www.gesetze-im-internet.de/stvo_2013/__37.html
- [S72] Hakozaki Junction (Shuto Expressway): https://en.wikipedia.org/wiki/Hakozaki_Junction
- [S73] TxDOT Roadway Design Manual 4.10.12, curbs (AASHTO curb types also in NAP 13849): https://www.txdot.gov/manuals/des/rdw/chapter-4--basic-design-criteria/4-10-cross-sectional-elements/4-10-12-curbs-and-curb-gutter.html
- [S74] OSM wiki, DE:Key:kerb (typical German kerb heights): https://wiki.openstreetmap.org/wiki/DE:Key:kerb
- [S75] Bavarian guideline on fire brigade areas (Schwabach copy): https://schwabach.feuerwehren.bayern/media/filer_public/b0/1b/b01b6f91-d169-4b60-a5ff-7e78868efaad/richtlinie-flaeche-feuerwehr.pdf
- [S76] Interstate Guide, design standards (enthusiast site, secondary): https://www.interstate-guide.com/design-standards/
- [S77] Osaka City document quoting the National Police Agency crosswalk criteria: https://www.city.osaka.lg.jp/ikuno/cmsfiles/contents/0000465/465369/06ooike08_sankou.pdf
- [S78] SCDOT roundabout criteria (NCHRP 672 ranges): https://www.scdot.org/business/pdf/I85/RoadwayDesign/Roundabout_Criteria.pdf
- [S79] Iowa DOT Design Manual 6C-5, turn lanes: https://iowadot.gov/design/dmanual/06c-05.pdf
- [S80] TxDOT Roadway Design Manual 4.10.2, speed change lanes (storage lengths): https://www.txdot.gov/manuals/des/rdw/chapter-4--basic-design-criteria/4-10-cross-sectional-elements/4-10-2-speed-change-lanes.html
- [S81] Virginia Transportation Research Council, report 94-TAR7 (dual left-turn lanes): https://vtrc.virginia.gov/reports/all-reports/94-tar7
- [S82] Wisconsin DOT Traffic Signal Design Manual 3-3-5 (opposing left-turn alignment): https://wisconsindot.gov/dtsdManuals/traffic-ops/manuals-and-standards/tsdm/03/03-03-05.pdf
- [S83] IRC 52:2019 hill road summary (secondary): https://infralens.in/code/IRC-52-2019
- [S84] Surrey County Council Healthy Streets 5.4 (Manual for Streets widths, fire access): https://healthystreets.surreycc.gov.uk/requirements-and-guidance/section?id=5.4
- [S85] City of Edinburgh street design guidance, PT3 bus priority: https://www.edinburgh.gov.uk/downloads/file/24986/pt-3---bus-priority

Notes on gaps (thin sources): the FGSV originals (RASt 06, RAA 2008, RAL 2012, ERA 2010) are paywalled, so
German values come from FGSV's own 2024 supplement and secondary documents; LTN 1/20 tables, CROW
widths, AASHTO Green Book tables (read through state DOT manuals), Alpine hairpin geometry rules, measured
real-world lane widths, Japanese crosswalk and turn-bay taper rules, esmini runtime behaviour at dead ends,
and an ASAM "junction guideline" document were not available as primary text.
