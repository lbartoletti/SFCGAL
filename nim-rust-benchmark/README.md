# Nim vs Rust Geometry Library Benchmark

Comparaison de performance et de lisibilité entre Nim et Rust pour une bibliothèque de géométrie GIS/CAD nD.

## Vue d'ensemble

Ce projet implémente plusieurs algorithmes de géométrie computationnelle identiques en **Nim** et **Rust** pour comparer :
- Les **performances** (temps d'exécution)
- La **lisibilité** du code
- L'**expressivité** des langages
- La **facilité de développement**

## Algorithmes implémentés

### 1. Triangulation de Delaunay
**Algorithme** : Delaunator (sweep-hull)
**Référence** : https://github.com/mapbox/delaunator
**Complexité** : O(n log n)

Triangulation de Delaunay rapide en 2D utilisant l'algorithme de balayage optimisé.

### 2. Enveloppe convexe 2D (Convex Hull)
**Algorithmes** :
- Andrew's Monotone Chain - O(n log n)
- Graham Scan - O(n log n)

Deux algorithmes classiques pour calculer l'enveloppe convexe de points en 2D.

### 3. Intersection de segments (Line Sweep)
**Algorithme** : Bentley-Ottmann
**Complexité** : O((n + k) log n) où k = nombre d'intersections

Algorithme de balayage pour trouver toutes les intersections entre segments de ligne.

### 4. Génération de maillage (Mesh)
**Fonctionnalités** :
- Génération de maillage triangulaire uniforme
- Raffinement de maillage basé sur la qualité
- Lissage Laplacien
- Métriques de qualité des triangles

### 5. Opérations d'overlay de polygones
**Algorithme** : Sutherland-Hodgman (pour intersection)
**Opérations** :
- Intersection de polygones
- Union de polygones
- Différence de polygones

## Architecture des types

### Nim
```nim
type
  Vector*[D: static[int], T] = object
    data*: array[D, T]

  Coordinate*[D: static[int], T] = distinct Vector[D, T]

  Point*[D: static[int], T] = object
    coord*: Coordinate[D, T]
```

Où :
- `D` : dimension (2, 3, 4, N)
- `T` : type numérique (int, float, fixed-point, fraction, etc.)

Séparation : `Vector` → `Coordinate` → `Point`

### Rust
```rust
pub struct Vector<const D: usize, T> {
    pub data: [T; D],
}

pub struct Coordinate<const D: usize, T>(pub Vector<D, T>);

pub struct Point<const D: usize, T> {
    pub coord: Coordinate<D, T>,
}
```

Utilisation de const generics pour la dimension à la compilation.

## Structure du projet

```
nim-rust-benchmark/
├── nim/
│   ├── geometry_types.nim    # Types de base
│   ├── delaunay.nim          # Triangulation de Delaunay
│   ├── convex_hull.nim       # Enveloppe convexe
│   ├── line_sweep.nim        # Intersection de segments
│   ├── mesh.nim              # Génération de maillage
│   ├── overlay.nim           # Overlay de polygones
│   └── benchmark.nim         # Suite de benchmarks
└── rust/
    ├── src/
    │   ├── geometry_types.rs # Types de base
    │   ├── delaunay.rs       # Triangulation de Delaunay
    │   ├── convex_hull.rs    # Enveloppe convexe
    │   ├── line_sweep.rs     # Intersection de segments
    │   ├── mesh.rs           # Génération de maillage
    │   ├── overlay.rs        # Overlay de polygones
    │   ├── lib.rs            # Module principal
    │   └── main.rs           # Suite de benchmarks
    └── Cargo.toml
```

## Compilation et exécution

### Nim

```bash
cd nim-rust-benchmark/nim
nim c -r -d:release --opt:speed benchmark.nim
```

Options de compilation :
- `-d:release` : mode release
- `--opt:speed` : optimisation pour la vitesse
- `--gc:arc` : utiliser le GC ARC (optionnel)

### Rust

```bash
cd nim-rust-benchmark/rust
cargo run --release
```

## Benchmarks

Les benchmarks testent les algorithmes avec différentes tailles d'entrées :

| Algorithme | Tailles testées |
|------------|-----------------|
| Delaunay | 100, 500, 1000, 5000 points |
| Convex Hull | 100, 500, 1000, 5000, 10000 points |
| Line Sweep | 50, 100, 200, 500 segments |
| Mesh | 100, 500, 1000, 2000 points |
| Polygon Overlay | 4, 8, 16, 32, 64 côtés |

## Comparaison de lisibilité

### Exemple : Cross Product 2D

**Nim** :
```nim
proc cross2D*(a, b: Coord2D): float64 {.inline.} =
  a.x * b.y - a.y * b.x
```

**Rust** :
```rust
pub fn cross2d(&self, other: &Self) -> f64 {
    self.x() * other.y() - self.y() * other.x()
}
```

### Exemple : Orientation Test

**Nim** :
```nim
proc orient2D*(a, b, c: Coord2D): float64 {.inline.} =
  cross2D(a, b, c)
```

**Rust** :
```rust
pub fn orient2d(a: &Coord2D, b: &Coord2D, c: &Coord2D) -> f64 {
    a.cross2d_with_origin(b, c)
}
```

## Points clés de comparaison

### Nim

**Avantages** :
- ✅ Syntaxe plus concise et lisible
- ✅ Métaprogrammation puissante avec macros
- ✅ Compilation rapide
- ✅ Pas de lifetime à gérer
- ✅ UFCS (Uniform Function Call Syntax)
- ✅ Syntaxe proche de Python

**Inconvénients** :
- ❌ Écosystème plus petit
- ❌ Moins de bibliothèques tierces
- ❌ Communauté plus petite
- ❌ Moins de tooling

### Rust

**Avantages** :
- ✅ Grande communauté active
- ✅ Écosystème riche (crates.io)
- ✅ Tooling excellent (cargo, clippy, rustfmt)
- ✅ Sécurité mémoire garantie
- ✅ Zero-cost abstractions
- ✅ Documentation exhaustive

**Inconvénients** :
- ❌ Syntaxe plus verbeuse
- ❌ Courbe d'apprentissage (ownership, lifetimes)
- ❌ Temps de compilation plus longs
- ❌ Borrow checker peut être frustrant

## Performance attendue

Les deux langages devraient offrir des performances similaires car :
- Compilation vers du code natif
- Pas de GC pendant les calculs (ARC/borrow checking)
- Optimisations LLVM (Rust) vs C backend (Nim)

Les différences de performance dépendront principalement de :
- L'efficacité des allocations mémoire
- La qualité des optimisations du compilateur
- L'utilisation des instructions SIMD

## Résultats des benchmarks

*À compléter après exécution des benchmarks*

### Delaunay Triangulation
| Points | Nim (ms) | Rust (ms) | Ratio |
|--------|----------|-----------|-------|
| 100    | -        | -         | -     |
| 500    | -        | -         | -     |
| 1000   | -        | -         | -     |
| 5000   | -        | -         | -     |

### Convex Hull (Andrew's Algorithm)
| Points | Nim (ms) | Rust (ms) | Ratio |
|--------|----------|-----------|-------|
| 100    | -        | -         | -     |
| 500    | -        | -         | -     |
| 1000   | -        | -         | -     |
| 5000   | -        | -         | -     |
| 10000  | -        | -         | -     |

## Conclusion

Ce benchmark vise à aider à la décision entre Nim et Rust pour une bibliothèque de géométrie nD professionnelle.

### Recommandations

**Choisir Nim si** :
- Vous privilégiez la rapidité de développement
- Vous voulez une syntaxe expressive et concise
- Vous n'avez pas besoin d'un large écosystème
- Vous développez principalement seul ou en petite équipe

**Choisir Rust si** :
- Vous avez besoin d'un écosystème riche
- Vous travaillez en grande équipe
- Vous voulez maximiser la sécurité et la fiabilité
- Vous avez besoin d'interopérabilité avec de nombreuses bibliothèques
- La communauté et le support sont critiques

## Prochaines étapes

1. Exécuter les benchmarks sur différentes plateformes
2. Optimiser les implémentations (SIMD, multi-threading)
3. Ajouter des tests unitaires complets
4. Implémenter des algorithmes 3D
5. Comparer la consommation mémoire

## Licence

À définir selon le projet parent.

## Auteur

Comparaison réalisée pour évaluer Nim vs Rust pour une bibliothèque GIS/CAD nD.
