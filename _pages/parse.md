---
layout: page
title: Constituency Parsing
keywords: parse, ParserAnnotator, constituency parsing
permalink: '/parse.html'
nav_order: 13
parent: Pipeline
---

## Description

Provides full syntactic analysis, minimally a constituency (phrase-structure tree) parse of sentences. If a rule-based conversion from constituency parses to dependency parses is available (this is currently the case for English and Chinese, only), then a dependency representation is also generated using this conversion. The constituent-based output is saved in TreeAnnotation. We generate three dependency-based outputs, as follows: basic dependencies, saved in BasicDependenciesAnnotation; enhanced dependencies saved in EnhancedDependenciesAnnotation; and enhanced++ dependencies in EnhancedPlusPlusDependenciesAnnotation. Most users of our parser will prefer the latter representation. Constituency parsers internally generate binary parse trees, which can also be saved.

If you only need dependency parses, then you can get only dependency parses more quickly (and using less memory) by using the direct dependency parser annotator `depparse`. Note that this is a separate annotator, with different options.

| Property name | Annotator class name | Generated Annotation |
| --- | --- | --- |
| parse | ParserAnnotator | TreeAnnotation, BasicDependenciesAnnotation, EnhancedDependenciesAnnotation, EnhancedPlusPlusDependenciesAnnotation, BinarizedTreeAnnotation, KBestTreesAnnotation |

## Options

| Option name | Type | Default | Description |
| --- | --- | --- | --- |
| parse.model | file, URL, or classpath resource | edu/stanford/nlp/models/lexparser/englishPCFG.ser.gz | Which parsing model to use. There is usually no need to explicitly set this option, unless you want to use a different parsing model than the default for a language, which is set in the language-particular CoreNLP properties file. You might change it to select a different kind of parser, or one suited to, e.g., caseless text. |
| parse.debug | boolean | false | Whether to print verbose messages while parsing. |
| parse.flags | String | -retainTmpSubcategories | Flags to use when loading the parser model.  The default for the English model is  "-retainTmpSubcategories"; other languages have an empty String default. The value is a whitespace separated list of flags and any arguments, just as would be passed on the command line. In particular, these are flags not properties, and an initial dash should be included. |
| parse.maxlen | integer | -1 | If set to a positive number, the annotator parses only sentences of length at most this number (in terms of number of tokens). For longer sentences, the parser creates a flat structure, where every token is assigned to the non-terminal X. This is useful when parsing noisy web text, which may generate arbitrarily long sentences. |
| parse.treemap | Class | null | If set to the String name of a Java class that is a Function&lt;Tree,Tree&gt;, then this function will be applied to each tree output by the parser. |
| parse.maxtime | long | 0 | This was supposed to be for a time-based cutoff for parsing, but it is currently unimplemented. |
| parse.kbest | integer | 0 | If a positive number, store the k-best parses in `KBestTreesAnnotation`. Note that this option only has an effect if you parse sentences with a PCFG model. |
| parse.originalDependencies | boolean | false | Generate original Stanford Dependencies grammatical relations instead of Universal Dependencies. Note, however, that some annotators that use dependencies such as `natlog` might not function properly if you use this option.  If you are using the [Neural Network dependency parser](http://nlp.stanford.edu/software/nndep.html) and want to get the original SD relations, see the [CoreNLP FAQ](faq.html#how-can-i-get-original-stanford-dependencies-instead-of-universal-dependencies) on how to use a model trained on Stanford Dependencies. |
| parse.buildgraphs | boolean | true | Generate dependency representations of the sentence, stored under the three Dependencies annotations mentioned in the introduction. |
| parse.nthreads | int | 1 | Number of threads to use for parsing. |
| parse.nosquash | boolean | false | If true, don't re-annotate sentences that already have a tree annotation. |
| parse.keepPunct | boolean | true | A boolean option on whether to keep punctuation depenencies in the dependency parse output of the parser. Starting from 2015, the default is `true`. |
| parse.extradependencies | { NONE, REF\_ONLY\_UNCOLLAPSED, REF\_ONLY\_COLLAPSED, SUBJ\_ONLY, REF\_UNCOLLAPSED\_AND\_SUBJ, REF\_COLLAPSED\_AND\_SUBJ, MAXIMAL  | NONE | A specification for the types of extra edges to add to the dependency tree for Stanford Dependencies. |
| parse.binaryTrees  | boolean | false | Whether to also store a binary version of the parse tree under `BinarizedTreeAnnotation`. |

*Note: The values of all options, in a Properties object or on the command-line, are of type String. The listed type says what kinds of values are appropriate and hence how the String will be parsed.*

## Caseless models

It is possible to run StanfordCoreNLP with a parser
model that ignores capitalization. We have trained models like this
for English. You can find details on the
[Caseless models](caseless.html) page.


## Shift-reduce parser

There is a much faster and more memory efficient parser available in
the shift reduce parser.  It takes quite a while to load, and the
download is much larger, which is the main reason it is not the
default.

Details on how to use it are available on the [shift reduce parser](http://nlp.stanford.edu/software/srparser.html) page.

## PoS tagging

All of our parsers make use of parts of speech. Some of the models (e.g., neural dependency parser and shift-reduce parser) require an external PoS tagger; you must specify the `pos` annotator. Other parsers, such as the PCFG and Factored parsers can either do their own PoS tagging or use an external PoS tagger as a preprocessor. If you want to use a parser as the PoS tagger, make sure you do *not* include `pos` in the list of annotators and position the annotator `parse` prior to any other annotator that requires part-of-speech information (such as `lemma`):
```
-annotators tokenize,ssplit,parse,lemma,ner
```
In general: these parsers are good PoS taggers; they are not quite as accurate as the supplied maxent PoS tagger in terms of overall token accuracy; however, they often do better in more “grammar-based” decision making, where broader syntactic context is useful, such as for distinguishing finite and non-finite verb forms.

## More information 

For more details on the original parsers, please see [this page](http://nlp.stanford.edu/software/lex-parser.html). 
There is also a page on the [shift reduce parser](http://nlp.stanford.edu/software/srparser.html).
For more details about Stanford dependencies, please refer to [this page](http://nlp.stanford.edu/software/stanford-dependencies.html).
A separate site documents [Universal Dependencies](http://universaldependencies.org/).

## Examples

* Find all of the `NP` and `VP` constituents in a parse tree

```java
package edu.stanford.nlp.examples;

import edu.stanford.nlp.ling.CoreAnnotations;
import edu.stanford.nlp.pipeline.Annotation;
import edu.stanford.nlp.pipeline.StanfordCoreNLP;
import edu.stanford.nlp.trees.*;

import java.util.*;

public class ConstituentExample {

  public static void main(String[] args) {
    // set up pipeline properties
    Properties props = new Properties();
    props.setProperty("annotators", "tokenize,ssplit,pos,lemma,ner,parse");
    // use faster shift reduce parser
    props.setProperty("parse.model", "edu/stanford/nlp/models/srparser/englishSR.ser.gz");
    props.setProperty("parse.maxlen", "100");
    // set up Stanford CoreNLP pipeline
    StanfordCoreNLP pipeline = new StanfordCoreNLP(props);
    // build annotation for a review
    Annotation annotation =
        new Annotation("The small red car turned very quickly around the corner.");
    // annotate
    pipeline.annotate(annotation);
    // get tree
    Tree tree =
        annotation.get(CoreAnnotations.SentencesAnnotation.class).get(0).get(TreeCoreAnnotations.TreeAnnotation.class);
    System.out.println(tree);
    Set<Constituent> treeConstituents = tree.constituents(new LabeledScoredConstituentFactory());
    for (Constituent constituent : treeConstituents) {
      if (constituent.label() != null &&
          (constituent.label().toString().equals("VP") || constituent.label().toString().equals("NP"))) {
        System.err.println("found constituent: "+constituent.toString());
        System.err.println(tree.getLeaves().subList(constituent.start(), constituent.end()+1));
      }
    }
  }
}
```
