---
layout: page
title: Quote Extraction And Attribution
keywords: quote, QuoteAnnotator, quote attribution
permalink: '/quote.html'
nav_order: 18
parent: Pipeline
---

## Description

Deterministically picks out quotes from a text. All top-level quotes, are supplied by the top level annotation for a text. If a QuotationAnnotation corresponds to a quote that contains embedded quotes, these quotes will appear as embedded QuotationAnnotations that can be accessed from the QuotationAnnotation that they are embedded in. The QuoteAnnotator can handle multi-line and cross-paragraph quotes, but any embedded quotes must be delimited by a different kind of quotation mark than its parents.

Allows for embedded quotations so long as they are either directed unicode quotes or are of a different type of quote than the outer quotations (e.g. "'Gadzooks' is what he said to me" is legal whereas "They called me "Danger" when I was..." is illegal). Uses regular-expression-like rules to find quotes and does not depend on the tokenizer, which allows quotes like ''Tis true!' to be correctly identified.

Considers regular ascii ("", '', \`\`'', and \`') as well as "smart" and international quotation marks as follows: “”,‘’, «», ‹›, 「」, 『』, „”, and ‚’.
 
Note: extracts everything within these pairs as a whole quote segment, which may or may not be the desired behaviour for texts that use different formatting styles than standard english ones.
 

| Property name | Annotator class name | Generated Annotation |
| --- | --- | --- |
| quote | QuoteAnnotator | QuotationsAnnotation, CharacterOffsetBeginAnnotation, CharacterOffsetEndAnnotation, QuotationIndexAnnotation, QuotationsAnnotation (if embedded quotes), TokensAnnotation (if tokenize present), TokenBeginAnnotation (if tokenize present), TokenEndAnnotation (if tokenize present), SentenceBeginAnnotation (if ssplit present), SentenceEndAnnotation (if ssplit present) | 

## Quote Attribution

As of version 3.9.1 of Stanford CoreNLP, the QuoteAnnotator will also try to attribute the quotes by default.

This can be deactivated by setting the `quote.attributeQuotes` property to `false.`

## Sample Command Line

If you run this command:

```bash
java -Xmx10g edu.stanford.nlp.pipeline.StanfordCoreNLP -annotators tokenize,ssplit,pos,lemma,ner,depparse,coref,quote -file quote-example.txt -outputFormat text
```

On this text `quote-example.txt`:

```
In the summer Joe Smith decided to go on vacation.  He said, "I'm going to Hawaii."  That July, vacationer Joe went to Hawaii.
```

You should get this output for the quote in the text:

```
Extracted quotes: 
[QuotationIndex=0, CharacterOffsetBegin=61, Text="I'm going to Hawaii.", Speaker=Joe Smith]
```

If you didn't want to run quote attribution, you would add `-quote.attributeQuotes false` to your command.


## Options

* quote.singleQuotes: "true" or "false", indicating whether or not to consider ' tokens (single quotes) to be quotation marks (default=false).
* quote.maxLength: maximum character length of quotes to consider (default=-1).
* quote.asciiQuotes: "true" or "false", indicating whether or not to convert all quotes to ascii quotes before processing (can help when there are errors in quote directionality) (default=false).
* quote.allowEmbeddedSame: "true" or "false" indicating whether or not to allow smart/directed (everything except " and ') quotes of the same kind to be embedded within one another (default=false).
