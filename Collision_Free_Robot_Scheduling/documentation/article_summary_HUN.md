# Collision-Free Robot Scheduling

**Szerzők: Duncan Adamson, Nathan Flaherty, Igor Potapov, Paul G. Spirakis**

## Abstract

Napjainkban egyre több laboratóriumi környezetben előfordulnak robotok. A cikk azt vizsgálja, hogy több, fix helyzetű
robot esetén hogyan lehet a kiosztott feladatokat megfelelően rendezni, hogy X feladatot végző robot ne blokkolja Y robot tevékenységét.

Hogy ezt illusztrálni lehessen, kell készíteni egy gráf-ot, melyen:
- Elhelyezünk node-okat, melyek a robotok által elvégezendő feladatokat illusztrálják
- Minden node-on pontosan egy robot állhat egy egységnyi időben
- Minden robatnak rendelkeznie kell egy "menetrenddel", melyben az van leírva, hogy mikor hol helyezkedik el a graph-on
- Amennyiben egy robot elfoglal egy node-ot, legalább annyi ideig ott kell maradnia, amennyi ideig tart a feladat
- Két node között való mozgás időbe tellik, azonban a robotok várakozhatnak egy node-on a fealadt elvégzése után
- Minden robotnak csak azokat a feladatokat kell teljesíteni, ami hozzá lett rendelve

Az algoritmus célja, hogy minden robotnak meghatározzunk egy olyan "menetrendet", melyben a lehető legkevesebb lépéssel végez az összes robot és minimalizáljuk az utolsónak végző robot haladási idejét.
Minden robot rendelkezni fog egy saját menetrenddel, mely időpillanatokból és elfoglalt helyből áll.

**Pl:**
```
Három robot esetén:
Robot 1: 5 egységnyi ideig fut
Robot 2: 7 egységnyi ideig fut
Robot 3: 10 egységnyi ideig fut

A tényleges futási idő 10 egység volt. A cél, hogy úgy alkossuk meg a "menetrendeket", hogy a 3. robot kevesebb idő alatt végezzen.
```

#### A képen feladatok és robotok találhatóak. A kékkel színezett node-ok jelentik a robotok által elfoglalt pozíciót, míg a pirossal színezett node-ok a feladatok helyét jelzik, ahol a piros szám a feladat hosszát jelöli

![image](https://github.com/user-attachments/assets/8309897a-296e-4f1f-8a4e-f003c896fee5)

*Kép kiemelve a cikkből*

A cikk bizonyítja, hogy teljes, csillag, "planar", "bipartite" gráfokra vetítve ez a probléma NP-complete.
Továbbá pozitív eredményeket mutat "line graph"-okra, miszerint található olyan optimális menetrend lista k mennyiségű robot számára,
m mennyiségű feladat esetén (ahol a feladatok ugyan olyan hosszúak és a feladatok közti átsorolási idő mindenhol n), mely
elvégezhető O(kmn) idő alatt. Illetve ad egy k-becslést, amikor a feladatok hossza határtalan.

## Összegzés

Fő feladat, hogy egy olyan "menetrend listát" találjunk, mely megfelel a következőknek:
- Minden feladat elkészül egy robot által
- 2 robot nem végezheti ugyanazt a feladatot egyszerre
- Minimalizálva lett a leghosszabb menetrend időtartama

### Főbb feladatok

- `NP-Completeness` be lett bizonyítva teljes, csillag, "planar" és "bipartite" gráfokra.
- Egy `optimális algoritmus` lett készítve a robotok renedezésére **útgráf** esetén, ahol minden feladat n egységnyi időt vesz igénybe
- Egy `k-becslés` algoritmus lett bemutatva olyan **útgráfok** esetén, melyeknek változó a feladata hosszúsága
- `k-ROBOT SCHEDULING` egyenlet készült különböző gráfokra

### Komplexitás eredmények

- `NP-complete`-nak nevezhető
    - teljes gráfok
    - csillag gráfok
    - Planar graphs
    - Bipartite graphs
- `Útgráfok`
    - **Optimális algoritmus** állapítható meg n hosszúságú feladatokk esetén
    - **k-becslés algoritmus** állapítható meg változó hosszúságú feladatok esetén

### Készített algoritmusok

- `Single robot scheduling`: Optimális egy robot esetén, útgráfon
- `Two robot scheduling`: Optimális és 2-becslés megoldások két robot esetén
- `k-robot scheduling`: Optimális egyenlő hosszúságú feladatok egy útgráf esetén és k-becslés határozható meg változó hosszúságú feladatok esetén

## Konklúzió

A cikk bemutatta, hogy ez valóban egy nehezen megoldható probléma még egyszerű gráfok esetén is.
Azonban megoldható útgráfok esetén, ahol n időt vesz igénybe az összes feladat. Becslés alkalmazható olyan
útgráfok esetén, ahol bármennyi időt felvehetnek a feladatok.

A cikk végén további nyitott kérdések merülnek fel:
- A cikk szerzői által készített algoritmust lehet-e optimalizálni vagy lehet-e készíteni optimális algoritmust
- Hogyan működne az algoritmus a tesztnek alá nem vetett gráfokon

## Szótár

- **Np-problem**
    - A problem is called NP (nondeterministic polynomial) if its solution can be guessed and verified in polynomial time; nondeterministic means that no particular rule is followed to make the guess. If a problem is NP and all other NP problems are polynomial-time reducible to it, the problem is NP-complete
- **Polynomial time**
    - An algorithm is said to be of polynomial time if its running time is upper bounded by a polynomial expression in the size of the input for the algorithm, that is, T(n) = O(nk) for some positive constant k
- **Planar Graph**
    - In graph theory, a planar graph is a graph that can be embedded in the plane, i.e., it can be drawn on the plane in such a way that its edges intersect only at their endpoints. In other words, it can be drawn in such a way that no edges cross each other.
- **Bipartite Graph**
    - In graph theory, a bipartite graph (or bigraph) is a graph whose vertices (or nodes) can be divided into two disjoint sets X and Y such that every edge connects a vertex in X to one in Y

## Fejlesztési lehetőségek
- A leírás angol változatának elkészítése
- Table of content generálása
