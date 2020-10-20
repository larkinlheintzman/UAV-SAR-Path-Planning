<map version="freeplane 1.6.0">
<!--To view this file, download free mind mapping software Freeplane from http://freeplane.sourceforge.net -->
<node TEXT="Planning for MRMH-SAR" FOLDED="false" ID="ID_1962365023" CREATED="1548337355793" MODIFIED="1548342107695" STYLE="oval">
<font SIZE="18"/>
<hook NAME="MapStyle" zoom="0.827">
    <properties edgeColorConfiguration="#808080ff,#ff0000ff,#0000ffff,#00ff00ff,#ff00ffff,#00ffffff,#7c0000ff,#00007cff,#007c00ff,#7c007cff,#007c7cff,#7c7c00ff" fit_to_viewport="false"/>

<map_styles>
<stylenode LOCALIZED_TEXT="styles.root_node" STYLE="oval" UNIFORM_SHAPE="true" VGAP_QUANTITY="24.0 pt">
<font SIZE="24"/>
<stylenode LOCALIZED_TEXT="styles.predefined" POSITION="right" STYLE="bubble">
<stylenode LOCALIZED_TEXT="default" ICON_SIZE="12.0 pt" COLOR="#000000" STYLE="fork">
<font NAME="SansSerif" SIZE="10" BOLD="false" ITALIC="false"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.details"/>
<stylenode LOCALIZED_TEXT="defaultstyle.attributes">
<font SIZE="9"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.note" COLOR="#000000" BACKGROUND_COLOR="#ffffff" TEXT_ALIGN="LEFT"/>
<stylenode LOCALIZED_TEXT="defaultstyle.floating">
<edge STYLE="hide_edge"/>
<cloud COLOR="#f0f0f0" SHAPE="ROUND_RECT"/>
</stylenode>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.user-defined" POSITION="right" STYLE="bubble">
<stylenode LOCALIZED_TEXT="styles.topic" COLOR="#18898b" STYLE="fork">
<font NAME="Liberation Sans" SIZE="10" BOLD="true"/>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.subtopic" COLOR="#cc3300" STYLE="fork">
<font NAME="Liberation Sans" SIZE="10" BOLD="true"/>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.subsubtopic" COLOR="#669900">
<font NAME="Liberation Sans" SIZE="10" BOLD="true"/>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.important">
<icon BUILTIN="yes"/>
</stylenode>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.AutomaticLayout" POSITION="right" STYLE="bubble">
<stylenode LOCALIZED_TEXT="AutomaticLayout.level.root" COLOR="#000000" STYLE="oval" SHAPE_HORIZONTAL_MARGIN="10.0 pt" SHAPE_VERTICAL_MARGIN="10.0 pt">
<font SIZE="18"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,1" COLOR="#0033ff">
<font SIZE="16"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,2" COLOR="#00b439">
<font SIZE="14"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,3" COLOR="#990000">
<font SIZE="12"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,4" COLOR="#111111">
<font SIZE="10"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,5"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,6"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,7"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,8"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,9"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,10"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,11"/>
</stylenode>
</stylenode>
</map_styles>
</hook>
<hook NAME="AutomaticEdgeColor" COUNTER="19" RULE="ON_BRANCH_CREATION"/>
<node TEXT="Human models" POSITION="left" ID="ID_1684848335" CREATED="1548337412756" MODIFIED="1548343523978">
<arrowlink SHAPE="CUBIC_CURVE" COLOR="#000000" WIDTH="2" TRANSPARENCY="200" FONT_SIZE="9" FONT_FAMILY="SansSerif" DESTINATION="ID_1876475106" STARTINCLINATION="-15;-95;" ENDINCLINATION="-330;-21;" STARTARROW="NONE" ENDARROW="DEFAULT"/>
<edge COLOR="#ff0000"/>
<node TEXT="Lost person model" ID="ID_260661044" CREATED="1548338990360" MODIFIED="1548338995115"/>
<node TEXT="Searcher model" ID="ID_1727449039" CREATED="1548338996496" MODIFIED="1548339002770"/>
</node>
<node TEXT="Terrain map" POSITION="left" ID="ID_1441035129" CREATED="1548337506683" MODIFIED="1548339052815">
<arrowlink SHAPE="CUBIC_CURVE" COLOR="#000000" WIDTH="2" TRANSPARENCY="200" FONT_SIZE="9" FONT_FAMILY="SansSerif" DESTINATION="ID_1876475106" STARTINCLINATION="28;-27;" ENDINCLINATION="-681;-114;" STARTARROW="NONE" ENDARROW="DEFAULT"/>
<edge COLOR="#ff00ff"/>
</node>
<node TEXT="Generate heatmap showing likelihood of the lost person having been at a location at a given time" POSITION="right" ID="ID_1876475106" CREATED="1548337624259" MODIFIED="1548337914559" HGAP_QUANTITY="25.249999664723884 pt" VSHIFT_QUANTITY="-6.74999979883433 pt">
<edge COLOR="#7c007c"/>
</node>
<node FOLDED="true" POSITION="right" ID="ID_1921489374" CREATED="1548337929203" MODIFIED="1548339290118" HGAP_QUANTITY="24.499999687075622 pt"><richcontent TYPE="NODE">

<html>
  <head>
    
  </head>
  <body>
    <p>
      Generate trajectories to optimize for <b>cost</b>&#160;while meeting <b>constraints</b>
    </p>
  </body>
</html>

</richcontent>
<edge COLOR="#7c7c00"/>
<node ID="ID_1843182780" CREATED="1548339153496" MODIFIED="1548339285583"><richcontent TYPE="NODE">

<html>
  <head>
    
  </head>
  <body>
    <p>
      <b>Cost</b>: Time / distance / energy expenditure
    </p>
    <p>
      <b>Constraints</b>: upper-bound the risk of misclassifying a &quot;cell&quot; given current data
    </p>
  </body>
</html>

</richcontent>
</node>
<node ID="ID_1244403626" CREATED="1548339153496" MODIFIED="1548339352243"><richcontent TYPE="NODE">

<html>
  <head>
    
  </head>
  <body>
    <p>
      <b>Constraint</b>: Maximum Time / distance / energy expenditure
    </p>
    <p>
      <b>Cost</b>: minimize the risk of misclassifying a &quot;cell&quot; given current data
    </p>
  </body>
</html>

</richcontent>
</node>
</node>
<node TEXT="Human searcher(s) and UAVs begin search" POSITION="right" ID="ID_20122805" CREATED="1548338010530" MODIFIED="1548343552960" HGAP_QUANTITY="27.499999597668662 pt" VSHIFT_QUANTITY="9.749999709427366 pt">
<edge COLOR="#ff0000"/>
</node>
<node TEXT="Information is collected during the search that points to the target having been at a location" POSITION="right" ID="ID_143645614" CREATED="1548338069210" MODIFIED="1548338965055" HGAP_QUANTITY="26.7499996200204 pt" VSHIFT_QUANTITY="5.999999821186071 pt">
<edge COLOR="#00ffff"/>
<node TEXT="Quantify new information and update heatmap accordingly" ID="ID_12597618" CREATED="1548338093873" MODIFIED="1548339623444" HGAP_QUANTITY="20.74999979883433 pt" VSHIFT_QUANTITY="14.249999575316918 pt">
<arrowlink SHAPE="CUBIC_CURVE" COLOR="#000000" WIDTH="2" TRANSPARENCY="200" FONT_SIZE="9" FONT_FAMILY="SansSerif" DESTINATION="ID_1288342923" STARTINCLINATION="86;0;" ENDINCLINATION="58;-33;" STARTARROW="NONE" ENDARROW="DEFAULT"/>
</node>
<node TEXT="Update trajectories" ID="ID_1488239750" CREATED="1548338134417" MODIFIED="1548339687502" HGAP_QUANTITY="19.999999821186073 pt" VSHIFT_QUANTITY="5.249999843537812 pt">
<arrowlink SHAPE="CUBIC_CURVE" COLOR="#000000" WIDTH="2" TRANSPARENCY="200" FONT_SIZE="9" FONT_FAMILY="SansSerif" DESTINATION="ID_847074053" STARTINCLINATION="208;0;" ENDINCLINATION="-140;-50;" STARTARROW="NONE" ENDARROW="DEFAULT"/>
</node>
</node>
<node TEXT="New information changes the prior generated by model (parametric updates)" POSITION="right" ID="ID_146741951" CREATED="1548338425856" MODIFIED="1548338972465" HGAP_QUANTITY="28.24999957531692 pt" VSHIFT_QUANTITY="10.499999687075624 pt">
<edge COLOR="#007c00"/>
<node TEXT="Reconstruct map with new prior and measurements so far" ID="ID_63790389" CREATED="1548338497942" MODIFIED="1548339614672">
<arrowlink SHAPE="CUBIC_CURVE" COLOR="#000000" WIDTH="2" TRANSPARENCY="200" FONT_SIZE="9" FONT_FAMILY="SansSerif" DESTINATION="ID_1288342923" STARTINCLINATION="53;0;" ENDINCLINATION="33;-32;" STARTARROW="NONE" ENDARROW="DEFAULT"/>
</node>
<node TEXT="Update trajectories" ID="ID_816464801" CREATED="1548338553520" MODIFIED="1548339691140" HGAP_QUANTITY="16.249999932944775 pt" VSHIFT_QUANTITY="5.999999821186071 pt">
<arrowlink SHAPE="CUBIC_CURVE" COLOR="#000000" WIDTH="2" TRANSPARENCY="200" FONT_SIZE="9" FONT_FAMILY="SansSerif" DESTINATION="ID_847074053" STARTINCLINATION="83;0;" ENDINCLINATION="-179;-29;" STARTARROW="NONE" ENDARROW="DEFAULT"/>
</node>
</node>
<node TEXT="Smoothing and Mapping" LOCALIZED_STYLE_REF="defaultstyle.floating" POSITION="right" ID="ID_1288342923" CREATED="1548339426987" MODIFIED="1548340239089" HGAP_QUANTITY="874.4999739378699 pt" VSHIFT_QUANTITY="223.49999333918112 pt">
<hook NAME="FreeNode"/>
<node TEXT="GraphSLAM, SAM, square-root SAM" ID="ID_813195303" CREATED="1548339528063" MODIFIED="1548342085909"/>
<node TEXT="iSAM, iSAM2, GTSAM" ID="ID_403995845" CREATED="1548342038634" MODIFIED="1548342099131">
<cloud COLOR="#ffff66" SHAPE="ARC"/>
</node>
</node>
<node TEXT="Continuous-time Gaussian motion planning" LOCALIZED_STYLE_REF="defaultstyle.floating" POSITION="right" ID="ID_847074053" CREATED="1548339648400" MODIFIED="1548342107695" HGAP_QUANTITY="874.4999739378698 pt" VSHIFT_QUANTITY="305.2499909028414 pt">
<hook NAME="FreeNode"/>
<node TEXT="GPMP" ID="ID_1922303414" CREATED="1548339717903" MODIFIED="1548341637759"/>
<node TEXT="GPMP2" ID="ID_957653490" CREATED="1548341620523" MODIFIED="1548341624999"/>
<node TEXT="iGPMP" FOLDED="true" ID="ID_1701369459" CREATED="1548341625883" MODIFIED="1548342346412">
<cloud COLOR="#ffff66" SHAPE="ARC"/>
<node TEXT="No global optimality guarantees" ID="ID_1791607412" CREATED="1548342162841" MODIFIED="1548342201907">
<node TEXT="Rapid random restarts?" ID="ID_211301315" CREATED="1548342317249" MODIFIED="1548342328980"/>
</node>
<node ID="ID_1815845281" CREATED="1548342204113" MODIFIED="1548342272429"><richcontent TYPE="NODE">

<html>
  <head>
    
  </head>
  <body>
    <p>
      Limited ability to handle nonlinear inequality constraints <i>(motion constraints)</i>
    </p>
  </body>
</html>

</richcontent>
<node ID="ID_738755480" CREATED="1548342278793" MODIFIED="1548342311822"><richcontent TYPE="NODE">

<html>
  <head>
    
  </head>
  <body>
    <p>
      Sequential Quadratic Programming <i>(SQP)</i>
    </p>
  </body>
</html>

</richcontent>
</node>
</node>
</node>
</node>
</node>
</map>
