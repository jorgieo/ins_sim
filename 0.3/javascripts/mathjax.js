// MathJax configuration for the ins_sim documentation.
//
// Every frame-indexed quantity in the docs is typeset through one of the
// macros below, so the Groves (2013) notation is defined in exactly one
// place. Frame-indexed macros take an optional first argument that
// decorates the symbol (\tilde = measured, \hat = estimated):
//
//   \Cmat{b}{n}          ->  C_b^n        (transformation from b to n)
//   \wvec{ib}{b}         ->  omega_ib^b   (angular rate of b w.r.t. i, in b axes)
//   \wvec[\tilde]{ib}{b} ->  measured omega_ib^b
//   \Cmat[\hat]{b}{n}    ->  estimated C_b^n
//   \skew{\wvec{ie}{n}}  ->  [omega_ie^n ^]  (skew-symmetric matrix)
//
// The full list, with examples, is in CONTRIBUTING.md (docs style guide).

window.MathJax = {
  loader: {
    // Load the extensions the notation depends on up front. Left to
    // MathJax's autoloader, they are fetched only when a formula first
    // uses them, and a typeset running alongside that fetch can render
    // the macro as undefined (red "\boldsymbol").
    load: ["[tex]/boldsymbol", "[tex]/color"],
  },
  tex: {
    packages: { "[+]": ["boldsymbol", "color"] },
    inlineMath: [["\\(", "\\)"]],
    displayMath: [["\\[", "\\]"]],
    processEscapes: true,
    processEnvironments: true,
    tags: "ams",
    macros: {
      // Coordinate transformation matrix C_beta^alpha (from beta to alpha)
      Cmat: ["#1{\\mathbf{C}}_{#2}^{#3}", 3, ""],
      // Kinematic vectors x_{beta alpha}^{gamma}
      wvec: ["#1{\\boldsymbol{\\omega}}_{#2}^{#3}", 3, ""],
      Wmat: ["#1{\\boldsymbol{\\Omega}}_{#2}^{#3}", 3, ""],
      fvec: ["#1{\\mathbf{f}}_{#2}^{#3}", 3, ""],
      vvec: ["#1{\\mathbf{v}}_{#2}^{#3}", 3, ""],
      rvec: ["#1{\\mathbf{r}}_{#2}^{#3}", 3, ""],
      gvec: ["#1{\\mathbf{g}}_{#2}^{#3}", 3, ""],
      qvec: ["#1{\\mathbf{q}}_{#2}^{#3}", 3, ""],
      // Attitude and velocity increments over one IMU interval
      avec: ["#1{\\boldsymbol{\\alpha}}_{#2}^{#3}", 3, ""],
      uvec: ["#1{\\boldsymbol{\\upsilon}}_{#2}^{#3}", 3, ""],
      // Attitude (small-angle) error vector, used as \delta\psivec{nb}{n}
      psivec: ["#1{\\boldsymbol{\\psi}}_{#2}^{#3}", 3, ""],
      // Skew-symmetric (cross-product) matrix [x ^]
      skew: ["\\left[#1\\wedge\\right]", 1],
      // 3x3 identity
      Ithree: "\\mathbf{I}_3",
    },
  },
  options: {
    ignoreHtmlClass: ".*|",
    processHtmlClass: "arithmatex",
  },
};

// MathJax typesets the first page itself during startup, so the initial
// document$ emission is skipped: typesetting it again here would run
// concurrently with that startup pass. If Material's instant navigation is
// enabled, later emissions (one per page swap) re-typeset the new content,
// chained on MathJax's own promise so that two typesets never overlap.
let initialDocument = true;
document$.subscribe(() => {
  if (initialDocument) {
    initialDocument = false;
    return;
  }
  MathJax.startup.promise = MathJax.startup.promise.then(() => {
    MathJax.startup.output.clearCache();
    MathJax.typesetClear();
    MathJax.texReset();
    return MathJax.typesetPromise();
  });
});
