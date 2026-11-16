const THEME_KEY = "theme";
const WIDE_QUERY = "(min-width: 1000px)";

export type Tab = "main" | "diagnostics" | "camera" | "system";

function readSavedTheme(): "light" | "dark" | null {
  try {
    const saved = localStorage.getItem(THEME_KEY);
    return saved === "light" || saved === "dark" ? saved : null;
  } catch {
    return null;
  }
}

class UiStore {
  dark = $state(false);
  wide = $state(false);
  tab = $state<Tab>("main");
  selectedModule = $state<string | null>(null);
  view = $state<"camera" | "topdown">("camera");

  start(): void {
    const saved = readSavedTheme();
    const systemDark = matchMedia("(prefers-color-scheme: dark)");
    this.dark = saved ? saved === "dark" : systemDark.matches;
    systemDark.addEventListener("change", (e) => {
      if (!readSavedTheme()) this.#applyTheme(e.matches);
    });
    this.#applyTheme(this.dark);

    const wide = matchMedia(WIDE_QUERY);
    this.wide = wide.matches;
    wide.addEventListener("change", (e) => (this.wide = e.matches));
  }

  setDark(dark: boolean): void {
    try {
      localStorage.setItem(THEME_KEY, dark ? "dark" : "light");
    } catch {}
    this.#applyTheme(dark);
  }

  showModule(id: string): void {
    this.selectedModule = id;
    this.tab = "diagnostics";
  }

  #applyTheme(dark: boolean): void {
    this.dark = dark;
    document.documentElement.dataset.theme = dark ? "dark" : "light";
    document
      .querySelector('meta[name="theme-color"]')
      ?.setAttribute("content", dark ? "#131312" : "#F2F1EC");
  }
}

export const ui = new UiStore();
