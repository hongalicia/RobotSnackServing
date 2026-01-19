import yaml
from pathlib import Path

def load_recipe(recipe_id: str):
    recipe_path = Path("recipes") / f"{recipe_id}.yaml"

    if not recipe_path.exists():
        raise FileNotFoundError(f"Recipe not found: {recipe_path}")

    with open(recipe_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    return data
