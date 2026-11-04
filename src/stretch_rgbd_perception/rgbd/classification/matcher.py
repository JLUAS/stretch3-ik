"""Small, inspectable known-label matcher using normalized Levenshtein distance."""

from dataclasses import dataclass
import unicodedata
import re

from rgbd.classification.products import LABELS


@dataclass
class Parameters:
    min_score: float = .75
    min_margin: float = .15
    min_ocr_confidence: float = .6


def normalize(text):
    text = unicodedata.normalize("NFKD", text.casefold())
    return "".join(c for c in text if c.isalnum() and not unicodedata.combining(c))


def similarity(left, right):
    row = list(range(len(right)+1))
    for i, a in enumerate(left, 1):
        next_row = [i]
        for j, b in enumerate(right, 1):
            next_row.append(min(next_row[-1]+1, row[j]+1, row[j-1]+(a != b)))
        row = next_row
    return 1-row[-1]/max(len(left), len(right), 1)


@dataclass(frozen=True)
class LabelMatch:
    label: str
    score: float
    margin: float
    accepted: bool
    ocr_confidence: float = 1.
    text: str = ''

    @property
    def quality(self):
        return self.score, self.ocr_confidence, self.margin


@dataclass(frozen=True)
class Observation:
    matches: tuple[LabelMatch, ...]

    @property
    def best(self):
        return self.matches[0] if len(self.matches) == 1 else None

    @property
    def ambiguous(self):
        return len(self.matches) > 1


class ProductMatcher:
    def __init__(self, labels=None, parameters=None):
        self.labels = dict(LABELS if labels is None else labels)
        self.parameters = Parameters() if parameters is None else parameters

    def match(self, text, confidence=1.):
        p = self.parameters
        if not self.labels or any(not 0 <= value <= 1 for value in (p.min_score, p.min_margin, p.min_ocr_confidence)):
            raise ValueError('Supply a dictionary and thresholds in [0, 1]')
        word = normalize(text)
        # Short fragments such as single letters provide too little evidence.
        if len(word) < 4:
            return None
        scores = sorted(((max(similarity(word, normalize(alias)) for alias in [name, *aliases]), name)
                         for name, aliases in self.labels.items()), reverse=True)
        score, name = scores[0]
        margin = score-(scores[1][0] if len(scores) > 1 else 0)
        return LabelMatch(name, score, margin,
                          score >= p.min_score and margin >= p.min_margin and confidence >= p.min_ocr_confidence,
                          confidence, text)

    def identify(self, readings):
        """One crop's direction hypotheses -> unique product evidence or ambiguity."""
        matches = {}
        for reading in readings:
            for text in (reading.text, *re.findall(r'\w+', reading.text)):
                match = self.match(text, reading.confidence)
                if match is not None and match.accepted and (
                        match.label not in matches or match.quality > matches[match.label].quality):
                    matches[match.label] = match
        return Observation(tuple(matches.values()))


if __name__ == '__main__':
    from rgbd.demos.detection import main
    main('products')
