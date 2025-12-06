# llm_agent.py
import os
import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel
from logger import get_logger

LOG = get_logger(__name__)
app = FastAPI()

OPENAI_KEY = os.environ.get("OPENAI_API_KEY")


class Req(BaseModel):
    board_state: dict
    available: list | None = []


def parse_piece_name(name: str):
    """
    Parse DEF names like:
      BIG_TRIANGLE_1_RED
      PARALELLOGRAM_2_BLUE
      SQUARE_RED
      SMALL_TRIANGLE_BLUE
    into (shape, color).

    This is purely string-based and does NOT need Webots imports.
    """
    parts = name.split("_")
    if not parts:
        return name, None

    color = parts[-1]  # assume last token is color: RED / BLUE / etc.
    shape = "_".join(parts[:-1]) if len(parts) > 1 else name
    return shape, color


@app.post("/recommend_piece")
def recommend_piece(req: Req):
    """
    Return { "recommended_piece": "<DEF_NAME>" }

    Uses a simple heuristic based only on board_state + available:
      - If nothing placed yet -> first available
      - Else, prefer an available piece whose shape matches something
        already placed, but with a different color (if possible).
      - Fallback: just return first available.
    """
    available = req.available or []
    placed = req.board_state.get("placed", [])

    LOG.info("LLM request: placed=%s available=%s", placed, available)

    if not available:
        LOG.warning("No available pieces given, returning None")
        return {"recommended_piece": None}

    # If nothing placed yet, just pick the first available
    if not placed:
        LOG.info("No pieces placed yet, choosing first available: %s", available[0])
        return {"recommended_piece": available[0]}

    # Build shape/color info for placed and available
    placed_shapes = {}
    for name in placed:
        shape, color = parse_piece_name(name)
        placed_shapes.setdefault(shape, set()).add(color)

    candidates = []
    for name in available:
        shape, color = parse_piece_name(name)
        # Prefer pieces whose shape is already on the board, but color is different
        if shape in placed_shapes:
            if color not in placed_shapes[shape]:
                candidates.append(name)

    if candidates:
        LOG.info("Heuristic chooses (shape match, diff color): %s", candidates[0])
        return {"recommended_piece": candidates[0]}

    # Fallback: just return first available
    LOG.info("Fallback: choosing first available: %s", available[0])
    return {"recommended_piece": available[0]}


if __name__ == "__main__":
    LOG.info("Starting llm_agent on http://127.0.0.1:8001")
    uvicorn.run("llm_agent:app", host="127.0.0.1", port=8001, reload=False)
